from typing import List

from as2_msgs.msg import YawMode
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose

from python_interface.behaviour_actions.gotowayp_behaviour import SendGoToWaypoint


class GotoGpsModule:
    __alias__ = "goto_gps"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

        self.__current_goto = None

    # def __call__(self, speed: float = 0.5) -> None:
    #     """Drone landing"""
    #     SendGoToWaypoint(self.__drone, float(speed))

    def __go_to(self, _x: float, _y: float, _z: float,
                speed: float, yaw_mode: int, yaw_angle: float, wait: bool = True) -> None:
        msg = GeoPose()
        msg.position.latitude = (float)(_x)
        msg.position.longitude = (float)(_y)
        msg.position.altitude = (float)(_z)
        SendGoToWaypoint(self, msg, speed, yaw_mode, yaw_angle, wait)

    def go_to_gps(self, lat: float, lon: float, alt: float, speed: float) -> None:
        """Go to GPS point (deg, m) with speed (m/s).

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        """
        self.__go_to(lat, lon, alt, speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def go_to_gps_with_yaw(self, lat: float, lon: float, alt: float, speed: float, angle: float) -> None:
        """Go to gps position with speed and angle

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        :type angle: float
        """
        self.__go_to(lat, lon, alt, speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def go_to_gps_path_facing(self, lat: float, lon: float, alt: float, speed: float) -> None:
        """Go to gps position with speed facing the goal

        :type lat: float
        :type lon: float
        :type alt: float
        :type speed: float
        """
        self.__go_to(lat, lon, alt, speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None)

    def go_to_gps_point(self, waypoint: List[float], speed: float) -> None:
        """Go to GPS point (deg, m) with speed (m/s).

        :type waypoint: List[float]
        :type speed: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)

    def go_to_gps_point_with_yaw(self, waypoint: List[float], speed: float, angle: float) -> None:
        """Go to gps point with speed and yaw angle

        :type waypoint: List[float]
        :type speed: float
        :type angle: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.FIXED_YAW, yaw_angle=angle)

    def go_to_gps_point_path_facing(self, waypoint: List[float], speed: float) -> None:
        """Go to gps point with speed facing the goal

        :type waypoint: List[float]
        :type speed: float
        """
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, yaw_mode=YawMode.PATH_FACING, yaw_angle=None)

    # TODO
    def __del__(self):
        del self.__drone.modules[self.__alias__]

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def modify(self, speed):
        raise NotImplementedError
