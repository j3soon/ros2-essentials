#!/usr/bin/env python3
import argparse
import math
import yaml
import pathlib
import urchin as urdf_loader
import pyrender
import warnings
import glob

import stretch_body.robot
import stretch_body.device
import stretch_body.hello_utils as hu
import time
import multiprocessing

try:
    # works on ubunut 22.04
    import importlib.resources as importlib_resources
    str(importlib_resources.files("stretch_body"))
except AttributeError as e:
    # works on ubuntu 20.04
    import importlib_resources
    str(importlib_resources.files("stretch_body"))

hu.print_stretch_re_use()
warnings.filterwarnings("ignore")

class URDFVisualizer:
    """The `show` method in this class is modified from the
    original implementation of `urdf_loader.URDF.show`. This class
    exists temporarily while the PR for this modification is
    in review.
    """

    def __init__(self, urdf):
        self.urdf = urdf
        self.nodes = None
        self.scene = None
        self.viewer = None

    def show(self, cfg=None, use_collision=False):
        """Visualize the URDF in a given configuration.
        Parameters
        ----------
        cfg : dict or (n), float
            A map from joints or joint names to configuration values for
            each joint, or a list containing a value for each actuated joint
            in sorted order from the base link.
            If not specified, all joints are assumed to be in their default
            configurations.
        use_collision : bool
            If True, the collision geometry is visualized instead of
            the visual geometry.
        """
        if use_collision:
            fk = self.urdf.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.urdf.visual_trimesh_fk(cfg=cfg)

        self.scene = pyrender.Scene(ambient_light = [0,0,0, 0.5])
        self.nodes = []
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            mesh_node = self.scene.add(mesh, pose=pose)
            self.nodes.append(mesh_node)
        self.viewer = pyrender.Viewer(self.scene, run_in_thread=True, use_raymond_lighting=True)

    def update_pose(self, cfg=None, use_collision=False):
        if use_collision:
            fk = self.urdf.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.urdf.visual_trimesh_fk(cfg=cfg)

        self.viewer.render_lock.acquire()
        for i, tm in enumerate(fk):
            pose = fk[tm]
            self.scene.set_pose(self.nodes[i], pose=pose)
        self.viewer.render_lock.release()



class StretchState:

    def __init__(self, robot, use_gripper,use_dw):
        self.stretch = robot
        self.use_gripper=use_gripper
        self.use_dw=use_dw

    def get_urdf_configuration(self):

        stretch_status = self.stretch.get_status()
        # set positions of the telescoping joints
        arm_status = stretch_status['arm']
        arm_m = arm_status['pos']
        telescoping_link_m = arm_m / 4.0

        lift_status = stretch_status['lift']
        lift_m = lift_status['pos']

        wrist_yaw_status = stretch_status['end_of_arm']['wrist_yaw']
        wrist_yaw_rad = wrist_yaw_status['pos']

        head_pan_status = stretch_status['head']['head_pan']
        head_pan_rad = head_pan_status['pos']

        head_tilt_status = stretch_status['head']['head_tilt']
        head_tilt_rad = head_tilt_status['pos']

        configuration = {
            'joint_left_wheel': 0.0,
            'joint_right_wheel': 0.0,
            'joint_lift': lift_m,
            'joint_arm_l0': telescoping_link_m,
            'joint_arm_l1': telescoping_link_m,
            'joint_arm_l2': telescoping_link_m,
            'joint_arm_l3': telescoping_link_m,
            'joint_wrist_yaw': wrist_yaw_rad,
            'joint_head_pan': head_pan_rad,
            'joint_head_tilt': head_tilt_rad
        }

        if self.use_gripper:
            try:
                gripper_status = stretch_status['end_of_arm']['stretch_gripper']
            except KeyError:
                print('Tool must include Stretch Gripper. Exiting...')
                exit(1)
            configuration['joint_gripper_finger_left'] = gripper_status['gripper_conversion']['finger_rad']/2
            configuration['joint_gripper_finger_right'] = gripper_status['gripper_conversion']['finger_rad']/2
        if self.use_dw:
            configuration['joint_wrist_pitch'] = stretch_status['end_of_arm']['wrist_pitch']['pos']
            configuration['joint_wrist_roll'] = stretch_status['end_of_arm']['wrist_roll']['pos']

        return configuration


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Python based URDF visualization')
    parser.add_argument('-t', "--trajectory", help="Visualize predefined trajectory", action="store_true")
    parser.add_argument('-c', "--collision", help="Use collision meshes", action="store_true")
    parser.add_argument('-g', "--gamepad", help="Use gamepad to control pose", action="store_true")
    args = parser.parse_args()

    pkg = str(importlib_resources.files("stretch_urdf"))  # .local/lib/python3.10/site-packages/stretch_urdf)
    models=['RE1V0','RE2V0','SE3']
    urdf_files=[]
    for m in models:
        urdf_files=urdf_files+glob.glob(pkg+'/'+m+'/*.urdf')
    print('-------------------')
    for i in range(len(urdf_files)):
        a=urdf_files[i]
        fn=a[a.find('stretch_description'):]
        print('%d : %s'%(i,fn))
    print('-------------------')
    id=-1
    while True:
        try:
            id=int(input('Enter ID of URDF to viz: '))
        except:
            pass
        if id>=0 and id<len(urdf_files):
            break
        print('Invalid entry')
    urdf_name=urdf_files[id]

    model_name=None
    for m in models:
        if urdf_name.find(m)>-1:
            model_name=m
    print('Using model: %s'%model_name)
    tool_name=urdf_name[urdf_name.rfind(model_name)+len(model_name)+1:-5]
    print('Using tool: %s'%tool_name)
    use_gripper=(tool_name=='tool_stretch_gripper' or tool_name=='tool_dex_wrist' or tool_name=='eoa_wrist_dw3_tool_sg3')
    use_dw = (tool_name=='tool_dex_wrist' or tool_name=='eoa_wrist_dw3_tool_sg3' or tool_name=='eoa_wrist_dw3_tool_nil' or tool_name=='eoa_wrist_dw3_tool_tablet_12in')

    urdf = urdf_loader.URDF.load(urdf_name)
    tool = stretch_body.device.Device(req_params=False).robot_params['robot']['tool']
    viz = URDFVisualizer(urdf)


    if args.trajectory:
        cfg_trajectory = {
            'joint_left_wheel': [0.0, math.pi],
            'joint_right_wheel': [0.0, math.pi],
            'joint_lift': [0.7, 0.7],
            'joint_arm_l0': [0.0, 0.0],
            'joint_arm_l1': [0.0, 0.0],
            'joint_arm_l2': [0.0, 0.0],
            'joint_arm_l3': [0.0, 0.0],
            'joint_wrist_yaw': [0, 0.0], #math.pi
            'joint_head_pan': [0.0, -(math.pi / 2.0)],
            'joint_head_tilt': [0.5, -0.5]
        }
        if use_gripper:
            cfg_trajectory['joint_gripper_finger_left']= [0.0, 0.25]
            cfg_trajectory['joint_gripper_finger_right']=[0.0, 0.25]
        if use_dw:
            cfg_trajectory['joint_wrist_pitch'] = [0, 0.0] #9math.pi
            cfg_trajectory['joint_wrist_roll'] = [0.0,math.pi]
        urdf.animate(cfg_trajectory=cfg_trajectory, use_collision=args.collision)
        exit()

    if args.gamepad:
        import stretch_body.gamepad_teleop
        viz_process_manager = multiprocessing.Manager()

        r = stretch_body.robot.Robot()
        r.startup()
        if not r.is_homed():
            print('Exiting because the robot has not been calibrated')
            exit()

        gamepad = stretch_body.gamepad_teleop.GamePadTeleop(robot_instance = False)
        gamepad.startup(robot=r)

        def _worker(viz_shared_cfg, viz_shared_collision):
            urdf = urdf_loader.URDF.load(urdf_name)
            tool = stretch_body.device.Device(req_params=False).robot_params['robot']['tool']
            viz = URDFVisualizer(urdf)
            collision = False
            if viz_shared_collision.get():
                collision = True
            # wait till the first joint config is updated from the main process
            time.sleep(1)
            viz.show(cfg=dict(viz_shared_cfg), use_collision=collision)
            while viz.viewer.is_active:
                viz.update_pose(cfg=dict(viz_shared_cfg), use_collision=collision)

        stretch_state = StretchState(r, use_gripper,use_dw)
        viz_shared_cfg = viz_process_manager.dict()
        viz_shared_collision = viz_process_manager.Value(typecode=bool,value=args.collision)


        viz_proccess = multiprocessing.Process(target=_worker,args=(viz_shared_cfg,viz_shared_collision,),daemon=True)
        viz_proccess.start()

        while True:
            try:
                cfg = stretch_state.get_urdf_configuration()
                for k in cfg.keys():
                    viz_shared_cfg[k] = cfg[k]
                gamepad.step_mainloop(r)
            except hu.ThreadServiceExit:
                print("Exiting...")
                viz_proccess.join()
                gamepad.gamepad_controller.stop()
                r.stop()
                exit()

    cfg_pose = {
        'joint_left_wheel': 0.0,
        'joint_right_wheel': 0.0,
        'joint_lift': 0.6,
        'joint_arm_l0': 0.1,
        'joint_arm_l1': 0.1,
        'joint_arm_l2': 0.1,
        'joint_arm_l3': 0.1,
        'joint_wrist_yaw': 0,
        'joint_head_pan': 0,
        'joint_head_tilt': 0
    }
    if use_gripper:
        cfg_pose['joint_gripper_finger_left'] = 0
        cfg_pose['joint_gripper_finger_right'] = 0
    if use_dw:
        cfg_pose['joint_wrist_pitch'] = 0
        cfg_pose['joint_wrist_roll'] = 0
    urdf.show(cfg=cfg_pose, use_collision=args.collision)




