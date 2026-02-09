from dataclasses import dataclass
from modbus_tk import modbus_tcp
import modbus_tk.defines as cst
from threading import Thread

import rospy


@dataclass
class ColisConsts:
    WIDTH_OUT = 16
    RG_BUSY_OUT = 17
    GRIP_DETECTED_OUT = 18
    WATCHDOG_OUT = 19
    ACTION_TRIGGER_IN = 20
    WIDTH_IN = 128
    FORCE_IN = 129


class Watchdog:
    WATCHDOG_TIMEOUT = rospy.Duration(0.2)

    def __init__(self, master: modbus_tcp.TcpMaster) -> None:
        self._master = master
        self._prev_state = False
        self._last_state_change_time = None

        self._watchdog_thread = Thread(target=self._update_watchdog_state, name='watchdog_thread')
        self._watchdog_thread.start()

        rospy.sleep(self.WATCHDOG_TIMEOUT)

    def _update_watchdog_state(self):
        rate = rospy.Rate(20.0)

        while True:
            state = bool(self._master.execute(1, cst.READ_COILS, ColisConsts.WATCHDOG_OUT, 1)[0])

            if state != self._prev_state:
                self._last_state_change_time = rospy.Time.now()

            self._prev_state = state
            rate.sleep()

    def is_active(self) -> bool:
        if self._last_state_change_time is None:
            return False
        
        return rospy.Time.now() - self._last_state_change_time < self.WATCHDOG_TIMEOUT


class Rg2Modbus:
    def __init__(self, ur_robot_ip: str) -> None:
        self._master = modbus_tcp.TcpMaster(host=ur_robot_ip, port=502)
        self._master.set_timeout(5.0)

        self.watchdog = Watchdog(self._master)    

    def send_gripp_req(self, width: float, force: float) -> None:
        self._check_modbus_connection()

        # set gripper width and force
        self._master.execute(
            1,
            cst.WRITE_MULTIPLE_REGISTERS,
            ColisConsts.WIDTH_IN,
            output_value=[int(width * 10), int(force * 10)],
        )

        # action trigger
        self._master.execute(
            1, cst.WRITE_SINGLE_COIL, ColisConsts.ACTION_TRIGGER_IN, output_value=1
        )
        rospy.sleep(0.05)
        self._master.execute(
            1, cst.WRITE_SINGLE_COIL, ColisConsts.ACTION_TRIGGER_IN, output_value=0
        )

    def querry_rg_busy_flag(self) -> bool:
        self._check_modbus_connection()
        return self._master.execute(1, cst.READ_COILS, ColisConsts.RG_BUSY_OUT, 1)[0]

    def querry_grip_detected_flag(self) -> bool:
        self._check_modbus_connection()
        return self._master.execute(1, cst.READ_COILS, ColisConsts.GRIP_DETECTED_OUT, 1)[0]

    def querry_gripper_width(self) -> float:
        self._check_modbus_connection()
        return self._master.execute(1, cst.READ_HOLDING_REGISTERS, ColisConsts.WIDTH_OUT, 1)[0] / 10
    
    def _check_modbus_connection(self):
        if not self.watchdog.is_active():
            raise ConnectionError(
                'The URscript has stopped running (watchdog is inactive).'
                'To restart it, use the VNC server or the Universal Robots ROS driver Dashboard.'
            )
