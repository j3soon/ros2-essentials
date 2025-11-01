"""
This is the implementation of the OGN node defined in SlCameraStreamer.ogn
"""
import carb
from dataclasses import dataclass
# imports needed if you want to create viewports of the camera
# from omni.kit.viewport.utility import get_viewport_from_window_name
# from omni.kit.viewport.utility import create_viewport_window
import omni.replicator.core as rep
from omni.isaac.core_nodes.bindings import _omni_isaac_core_nodes
from omni.isaac.core.prims import XFormPrim
from pxr import Vt
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_at_path
from omni.isaac.sensor import _sensor
from sl.sensor.camera.pyzed_sim_streamer import ZEDSimStreamer, ZEDSimStreamerParams 
import time
import copy
from sl.sensor.camera.ogn.SlCameraStreamerDatabase import SlCameraStreamerDatabase

STREAM_PORT = 30000
LEFT_CAMERA_PATH = "/base_link/ZED_X/CameraLeft"
RIGHT_CAMERA_PATH = "/base_link/ZED_X/CameraRight"
IMU_PRIM_PATH = "/base_link/ZED_X/Imu_Sensor"
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
CHANNELS = 3
    
class SlCameraStreamer:
    """
         Streams camera data to the ZED SDK
    """


    @dataclass
    class State:
        initialized: bool = False
        render_product_left = None
        render_product_right = None
        annotator_left = None
        annotator_right = None
        core_nodes_interface = None
        pyzed_streamer = None
        start_time = -1.0
        last_timestamp = 0.0
        camera_prim_name = None
        override_simulation_time = False
        imu_sensor_interface = None
        imu_prim_path = ""
        invalid_images_count = 0
        pass

    @staticmethod
    def internal_state() -> State:
        return SlCameraStreamer.State()

    @staticmethod
    def get_render_product_path(camera_path :str, render_product_size = [IMAGE_WIDTH, IMAGE_HEIGHT], force_new=True):
        """Helper function to get render product path

        Args:
            camera_path (str): the path of the camera prim
            render_product_size (list, optional): the resolution of the image. Defaults to [IMAGE_WIDTH, IMAGE_HEIGHT].
            force_new (bool, optional): forces the creation of a new render product. Defaults to True.

        Returns:
            str: the created render product 
        """
        render_product_path = ""
        render_product_path = rep.create.render_product(camera_path, render_product_size, force_new=force_new)
        return render_product_path
    
    @staticmethod
    def get_image_data(annotator):
        """Helper function to fetch image data.

        Args:
            annotator: the annotator to fetch the data

        Returns:
            3D array: the image data, or None if unsuccessful
            bool: True if successful, false otherwise
        """
        data = annotator.get_data()
        result = True
        try:
            if data is None:
                result = False
                return data, result
            data = data[:, :, :3]
            if not data.shape == (IMAGE_HEIGHT, IMAGE_WIDTH, CHANNELS):
                result = False
        except:
            return None, False
        return data, result
    
    @staticmethod
    def check_camera(camera_prim_path : str):
        result = False
        default_fl, default_apt_h, default_apt_v = 2.208, 5.76, 3.24
        default_projection_type = "pinhole"
        allowed_error = 0.01
        if is_prim_path_valid(camera_prim_path) == True:
            cam_prim = get_prim_at_path(prim_path=camera_prim_path)
            fl = cam_prim.GetAttribute("focalLength")
            apt_h = cam_prim.GetAttribute("horizontalAperture")
            apt_v = cam_prim.GetAttribute("verticalAperture")
            projection = cam_prim.GetAttribute("cameraProjectionType")
            result = True
            projection_type = projection.Get()
            if projection_type and ((abs(fl.Get() - default_fl) >= allowed_error)
                or (abs(apt_h.Get() - default_apt_h) >= allowed_error)
                or (abs(apt_v.Get() - default_apt_v) >= allowed_error)):
                fl.Set(default_fl)
                apt_h.Set(default_apt_h)
                apt_v.Set(default_apt_v)
                projection.Set(Vt.Token(default_projection_type))
                carb.log_warn(f"Camera {camera_prim_path} properties are not valid. Setting them back to default value.")
                result = True
        return result


    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""
        if db.internal_state.initialized is False:
            db.internal_state.core_nodes_interface = _omni_isaac_core_nodes.acquire_interface()
            db.internal_state.imu_sensor_interface =  _sensor.acquire_imu_sensor_interface()
            if db.inputs.camera_prim is None:
                carb.log_error("Invalid Camera prim")
                return
            
            # Check port
            port = db.inputs.streaming_port
            if  port <= 0 or port %2 == 1:
                carb.log_warn("Invalid port passed! It must be a positive even number. Will default to 30000.")
                port = 30000
            
            db.internal_state.override_simulation_time = db.inputs.use_system_time
            if db.internal_state.override_simulation_time:
                carb.log_warn("Overriding simulation time by system time")
            
            if not len(db.inputs.camera_prim) == 1:
                carb.log_error("Please pass the correct target to the omnigraph node")
                return False
            db.internal_state.camera_prim_name = db.inputs.camera_prim[0].name

            left_cam_path = db.inputs.camera_prim[0].pathString + LEFT_CAMERA_PATH
            right_cam_path = db.inputs.camera_prim[0].pathString + RIGHT_CAMERA_PATH
            res = SlCameraStreamer.check_camera(left_cam_path)
            res = res and SlCameraStreamer.check_camera(right_cam_path)
            if not res:
                carb.log_warn(f"[{db.inputs.camera_prim[0].GetPrimPath()}] Invalid or non existing zed camera, try to re-import your camera prim.")

            if (db.internal_state.annotator_left is None):
                render_product_path_left = SlCameraStreamer.get_render_product_path(left_cam_path)
                db.internal_state.annotator_left = rep.AnnotatorRegistry.get_annotator("rgb")
                db.internal_state.annotator_left.attach([render_product_path_left])

            if (db.internal_state.annotator_right is None):
                render_product_path_right = SlCameraStreamer.get_render_product_path(right_cam_path)
                db.internal_state.annotator_right = rep.AnnotatorRegistry.get_annotator("rgb")
                db.internal_state.annotator_right.attach([render_product_path_right])

            db.internal_state.imu_prim_path = db.inputs.camera_prim[0].pathString + IMU_PRIM_PATH

            # Setup streamer parameters
            db.internal_state.pyzed_streamer = ZEDSimStreamer()

            # Check serial number
            serial_number = db.inputs.serial_number
            camera_ids = db.internal_state.pyzed_streamer.getVirtualCameraIdentifiers()
            if serial_number not in camera_ids:
                serial_number = next(iter(camera_ids))
                carb.log_warn(f"Invalid serial number passed. Defaulting to: {serial_number}.")

            streamer_params = ZEDSimStreamerParams()
            streamer_params.image_width = IMAGE_WIDTH
            streamer_params.image_height = IMAGE_HEIGHT
            streamer_params.alpha_channel_included = False
            streamer_params.rgb_encoded = True
            streamer_params.serial_number = serial_number
            streamer_params.port = port
            streamer_params.codec_type = 1
            db.internal_state.pyzed_streamer.init(streamer_params)

            # set state to initialized
            carb.log_info(f"Streaming camera {db.internal_state.camera_prim_name} at port {port} and using serial number {serial_number}.")
            db.internal_state.invalid_images_count = 0
            db.internal_state.initialized = True

        try:
            ts : int = 0
            if db.internal_state.initialized is True:
                left, right = None, None
                current_time = db.internal_state.core_nodes_interface.get_sim_time()
                
                # we fetch image data from annotators - we do it sequentially to avoid fetching both if one fails
                left, res = SlCameraStreamer.get_image_data(db.internal_state.annotator_left)
                if res == False:
                    db.internal_state.invalid_images_count +=1
                    if db.internal_state.invalid_images_count >= 10:
                        carb.log_warn(f"{db.internal_state.camera_prim_name} - Left camera retrieved unexpected "
                                      "data shape, skipping frame.")
                    return False # no need to continue compute
                
                right, res = SlCameraStreamer.get_image_data(db.internal_state.annotator_right)
                if res == False:
                    db.internal_state.invalid_images_count +=1
                    if db.internal_state.invalid_images_count >= 10:
                        carb.log_warn(f"{db.internal_state.camera_prim_name} - Right camera retrieved unexpected "
                                      "data shape, skipping frame.")
                    return False # no need to continue compute
                else:
                    db.internal_state.invalid_images_count = 0 # reset counter once both frame were grabbed

                # Fetch simulation time and change its reference point to (system time at start)
                if db.internal_state.start_time == -1:
                    carb.log_info(f"{db.internal_state.camera_prim_name} - Starting stream to the ZED SDK")
                if db.internal_state.start_time == -1 or current_time < db.internal_state.last_timestamp:
                    db.internal_state.start_time = int(time.time() * 1000)
                    carb.log_info(f"{db.internal_state.camera_prim_name} - Setting initial streaming time stamp to: {db.internal_state.start_time}")
                ts = int(db.internal_state.start_time + current_time * 1000)
                
                # override with system time
                if db.internal_state.override_simulation_time:
                    ts = int(time.time() * 1000)

                # fetch IMU data if the imu prim is there - this check allows user to basically delete
                # their IMUand still have access to the image functionality without issues in Isaac Sim
                lin_acc_x, lin_acc_y, lin_acc_z = 0, 0, 0
                orientation = [0]*4
                if is_prim_path_valid(db.internal_state.imu_prim_path) == True:
                    imu_prim = XFormPrim(prim_path=db.internal_state.imu_prim_path)
                    temp_orientation = imu_prim.get_world_pose()[1]
                    swp = copy.deepcopy(temp_orientation)
                    orientation = [swp[0], -swp[1], -swp[3], -swp[2]]
                    if(orientation[0] == 0 and orientation[1] == 0 and  orientation[2] == 0 and orientation[3] == 0):
                        carb.log_warn(f"{db.internal_state.camera_prim_name} - Received invalid orientation, skipped frame !")
                        return False
                    if imu_prim.is_valid() == True:
                        if db.internal_state.imu_sensor_interface.is_imu_sensor(db.internal_state.imu_prim_path) == True:
                            imu_reading = db.internal_state.imu_sensor_interface.get_sensor_reading(db.internal_state.imu_prim_path)
                            if imu_reading.is_valid:
                                lin_acc_x = imu_reading.lin_acc_x
                                lin_acc_y = imu_reading.lin_acc_y
                                lin_acc_z = imu_reading.lin_acc_z
                
                # stream data
                if not (left is None) and not (right is None):
                    res = db.internal_state.pyzed_streamer.stream(
                        left.tobytes(),
                        right.tobytes(),
                        ts,
                        orientation[0],
                        orientation[1],
                        orientation[2],
                        orientation[3],
                        lin_acc_x,
                        lin_acc_y,
                        lin_acc_z,
                    )
                    if not res == 0:
                        if db.internal_state.invalid_images_count >= 10:
                            carb.log_warn(f"{db.internal_state.camera_prim_name} - Streaming failed at timestamp {ts} with error code: {res}")
                        db.internal_state.invalid_images_count += 1
                    else:
                        carb.log_verbose(f"Streamed at {ts} 2 rgb images and orientation: {orientation} ")
                    db.internal_state.last_timestamp = current_time
                    

        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True

    @staticmethod
    def release(node):
        carb.log_verbose("Releasing resources")        
        try:
            state = SlCameraStreamerDatabase.per_node_internal_state(node)
            if state is not None:
                # disabling this could fix the render product issue (return empty arrays) when the scene reloads
                # but could also create issues when attaching cameras to render products
                if state.annotator_left:
                    state.annotator_left.detach([state.render_product_left])
                    if state.render_product_left is not None:
                        state.render_product_left.destroy()
                if state.annotator_right:
                    state.annotator_right.detach([state.render_product_right])
                    if state.render_product_right is not None:
                        state.render_product_right.destroy()
                
                if state.pyzed_streamer:
                    state.pyzed_streamer.close()
                    # remove the streamer object when no longer needed
                    del state.pyzed_streamer
                _sensor.release_imu_sensor_interface(state.imu_sensor_interface)
                state.initialized = False
            
        except Exception:
            state = None
            pass
