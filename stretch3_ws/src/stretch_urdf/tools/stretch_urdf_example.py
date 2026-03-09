#! /usr/bin/env python3


import urchin as urdf_loader
import time
import stretch_body.robot
try:
    # works on ubunut 22.04
    import importlib.resources as importlib_resources
    str(importlib_resources.files("stretch_body"))
except AttributeError as e:
    # works on ubuntu 20.04
    import importlib_resources
    str(importlib_resources.files("stretch_body"))


def get_configuration(robot):
    s = robot.get_status()
    configuration = {
        'joint_left_wheel': 0.0,
        'joint_right_wheel': 0.0,
        'joint_lift': s['lift']['pos'],
        'joint_arm_l0': s['arm']['pos'] / 4.0,
        'joint_arm_l1': s['arm']['pos'] / 4.0,
        'joint_arm_l2': s['arm']['pos'] / 4.0,
        'joint_arm_l3': s['arm']['pos'] / 4.0,
        'joint_wrist_yaw': s['end_of_arm']['wrist_yaw']['pos'],
        'joint_head_pan': s['head']['head_pan']['pos'],
        'joint_head_tilt': s['head']['head_tilt']['pos']}
    return configuration

def main():
    r = stretch_body.robot.Robot()
    r.startup()

    pkg = str(importlib_resources.files("stretch_urdf"))  # eg .local/lib/python3.10/site-packages/stretch_urdf)
    model_name = r.params['model_name']
    tool_name = r.params['tool']
    urdf_name = pkg + '/%s/stretch_description_%s_%s.urdf' % (model_name, model_name, tool_name)
    urdf = urdf_loader.URDF.load(urdf_name)
    links = ['base_link','link_wrist_yaw']

    try:
        while True:
            lfk = urdf.link_fk(cfg=get_configuration(r), links=links, use_names=True)
            print('###############################3')
            for ll in links:
                print('----------- %s ---------'%ll.upper())
                print(lfk[ll])
            time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    r.stop()

if __name__ == "__main__":
    main()