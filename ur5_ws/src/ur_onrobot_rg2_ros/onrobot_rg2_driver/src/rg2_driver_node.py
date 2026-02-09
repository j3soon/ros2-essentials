#!/usr/bin/python3
from modbus_tk.modbus import ModbusError
from threading import Lock

import rospy
from rospy import Duration

from sensor_msgs.msg import JointState

from onrobot_rg2_driver.srv import GripperState, GripperStateResponse, GripperStateRequest

from rg2_modbus import Rg2Modbus


class Rg2DriverNode:
    MIN_RG2_FORCE: float = 3.0  # [N]
    MAX_RG2_FORCE: float = 40.0  # [N]
    MIN_RG2_RANGE: float = 0.0  # [N]
    MAX_RG2_RANGE: float = 100.0  # [N]

    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        ur_robot_ip = rospy.get_param('~ur_robot_ip', '10.15.20.4')
        frame_id = rospy.get_param('~base_link_frame', 'rg2_base_link')
        joint_name = rospy.get_param('~gripper_joint_name', 'rg2_gripper_joint')
        self._service_call_timeout = rospy.Duration(
            float(rospy.get_param('~service_call_timeout_sec', 10.0))
        )
        self._lock = Lock()

        self._rg2_modbus = Rg2Modbus(ur_robot_ip)

        self._joint_state_msg = JointState()
        self._joint_state_msg.header.frame_id = frame_id
        self._joint_state_msg.name = [joint_name]

        rate = rospy.Rate(0.5)
        while not self._rg2_modbus.watchdog.is_active():
            rospy.loginfo_throttle(5.0, 'Waiting for URscript Modbus watchdog activation...')
            rate.sleep()

        self._joint_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)

        self._set_gripp_state_service = rospy.Service(
            "rg2/set_gripper_width", GripperState, self._set_grip_state
        )

        rospy.Timer(rospy.Duration(0.05), self._pub_joint_state_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _set_grip_state(self, req: GripperStateRequest) -> GripperStateResponse:
        with self._lock:
            try:
                if not self.MIN_RG2_FORCE <= req.force <= self.MAX_RG2_FORCE:
                    raise ValueError(
                        f'Force value must be between {self.MIN_RG2_FORCE} and {self.MAX_RG2_FORCE} [N]'
                    )
                if not self.MIN_RG2_RANGE <= req.width <= self.MAX_RG2_RANGE:
                    raise ValueError(
                        f'Width value must be between {self.MIN_RG2_RANGE} and {self.MAX_RG2_RANGE} [mm]'
                    )

                rospy.loginfo(
                    f'[{rospy.get_name()}] Sending gripper request:\n'
                    f'\t - width: {req.width}\n\t - force: {req.force}'
                )

                self._rg2_modbus.send_gripp_req(req.width, req.force)
                grip_detected = self._wait_for_grip_completion(self._service_call_timeout)

                rospy.loginfo(f'[{rospy.get_name()}] Gripper action success!')

                return GripperStateResponse(True, grip_detected, None)

            except (ValueError, TimeoutError, ModbusError, ConnectionError) as e:
                rospy.logwarn(f'[{rospy.get_name()}] Gripper action failed: {str(e)}')
                return GripperStateResponse(False, False, str(e))

    def _wait_for_grip_completion(self, timeout: Duration) -> bool:
        start_time = rospy.Time.now()
        rate = rospy.Rate(20.0)

        while not self._rg2_modbus.querry_rg_busy_flag():
            if rospy.Time.now() - start_time > timeout / 10.0:
                # The preset position of the gripper is the same as
                # the actual position before the service was sent
                return self._rg2_modbus.querry_grip_detected_flag()
            rate.sleep()

        rospy.loginfo(f'[{rospy.get_name()}] Request sent successfully.')

        while self._rg2_modbus.querry_rg_busy_flag():
            if rospy.Time.now() - start_time > timeout:
                raise TimeoutError('Gripper action failed: timeout')
            rate.sleep()

        return self._rg2_modbus.querry_grip_detected_flag()

    def _pub_joint_state_cb(self, *args) -> None:
        try:
            self._joint_state_msg.header.stamp = rospy.Time.now()

            rg_width = self._rg2_modbus.querry_gripper_width()
            joint_state = self._map(rg_width, 0.0, 101.0, -0.44, 1.0)
            self._joint_state_msg.position = [joint_state]

            self._joint_publisher.publish(self._joint_state_msg)

        except ConnectionError as e:
            rospy.logerr_throttle(20.0, str(e))

    @staticmethod
    def _map(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min


def main():
    rg2_driver_node = Rg2DriverNode('rg2_driver_node')
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
