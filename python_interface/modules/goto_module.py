from typing import List

from as2_msgs.msg import YawMode
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose

from python_interface.behaviour_actions.gotowayp_behaviour import SendGoToWaypoint


class GotoModule:
    __alias__ = "goto"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

        self.__current_goto = None

    def __call__(self, _x: float, _y: float, _z: float, speed: float,
                 yaw_mode: int = YawMode.FIXED_YAW, yaw_angle: float = None, wait: bool = True) -> None:
        """Go to point (m) with speed (m/s).

        :type _x: float
        :type _y: float
        :type _z: float
        :type speed: float
        :type yaw_mode: int
        :type yaw_angle: float
        :type wait: bool
        """
        self.__go_to(_x, _y, _z, speed, yaw_mode, yaw_angle, wait)

    def __go_to(self, _x: float, _y: float, _z: float,
                speed: float, yaw_mode: int, yaw_angle: float, wait: bool = True) -> None:
        msg = Pose()
        msg.position.x = (float)(_x)
        msg.position.y = (float)(_y)
        msg.position.z = (float)(_z)
        self.__current_goto = SendGoToWaypoint(self, msg, speed, yaw_mode, yaw_angle, wait)

    # Method simplifications
    def go_to(self, _x: float, _y: float, _z: float, speed: float) -> None:
        """Go to point (m) with speed (m/s).

        :type _x: float
        :type _y: float
        :type _z: float
        :type speed: float
        """
        self.__go_to(_x, _y, _z, speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def go_to_with_yaw(self, _x: float, _y: float, _z: float, speed: float, angle: float) -> None:
        """Go to position with speed and yaw_angle

        :type _x: float
        :type _y: float
        :type _z: float
        :type speed: float
        :type yaw_angle: float
        """
        self.__go_to(_x, _y, _z, speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def go_to_path_facing(self, _x: float, _y: float, _z: float, speed: float) -> None:
        """Go to position facing goal with speed

        :type _x: float
        :type _y: float
        :type _z: float
        :type speed: float
        """
        self.__go_to(_x, _y, _z, speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None)

    def go_to_point(self, point: List[float], speed: float) -> None:
        """Go to point (m) with speed (m/s).

        :type point: List[float]
        :type speed: float
        """
        self.__go_to(point[0], point[1], point[2],
                     speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def go_to_point_with_yaw(self, point: List[float], speed: float, angle: float) -> None:
        """Go to point with speed and yaw_angle

        :type point: List[float]
        :type speed: float
        :type ignore_yaw: bool, optional
        """
        self.__go_to(point[0], point[1], point[2],
                     speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def go_to_point_path_facing(self, point: List[float], speed: float) -> None:
        """Go to point facing goal with speed

        :type point: List[float]
        :type speed: float
        """
        self.__go_to(point[0], point[1], point[2],
                     speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None)

    # TODO
    def __del__(self):
        del self.__drone.modules[self.__alias__]

    def pause(self) -> None:
        raise NotImplementedError

    def resume(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        """Stops goto if running
        """
        if self.__current_goto:
            self.__current_goto.stop()

    def modify(self, _x: float, _y: float, _z: float,
                speed: float, yaw_mode: int, yaw_angle: float) -> None:
        # msg = Pose()
        # msg.position.x = (float)(_x)
        # msg.position.y = (float)(_y)
        # msg.position.z = (float)(_z)
        # self.__current_goto.modify(msg)
        raise NotImplementedError
