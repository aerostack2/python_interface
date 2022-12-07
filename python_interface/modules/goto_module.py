from typing import List

from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPose

from python_interface.behaviour_actions.gotowayp_behaviour import SendGoToWaypoint


class GotoModule:
    __alias__ = "goto"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

    # def __call__(self, speed: float = 0.5) -> None:
    #     """Drone landing"""
    #     SendGoToWaypoint(self.__drone, float(speed))

    def __go_to(self, _x: float, _y: float, _z: float,
                speed: float, ignore_yaw: bool, is_gps: bool) -> None:
        if is_gps:
            msg = GeoPose()
            msg.position.latitude = (float)(_x)
            msg.position.longitude = (float)(_y)
            msg.position.altitude = (float)(_z)
        else:
            msg = Pose()
            msg.position.x = (float)(_x)
            msg.position.y = (float)(_y)
            msg.position.z = (float)(_z)
        SendGoToWaypoint(self, msg, speed, ignore_yaw)

    def go_to(self, _x: float, _y: float, _z: float, speed: float, ignore_yaw: bool = True) -> None:
        """Drone go to"""
        self.__go_to(_x, _y, _z, speed, ignore_yaw, is_gps=False)

    def go_to_point(self, point: List[float],
                    speed: float, ignore_yaw: bool = True) -> None:
        """Drone go to"""
        self.__go_to(point[0], point[1], point[2],
                     speed, ignore_yaw, is_gps=False)

    def go_to_gps(self, lat: float, lon: float, alt: float,
                  speed: float, ignore_yaw: bool = True) -> None:
        """Drone go to gps pose"""
        self.__go_to(lat, lon, alt, speed, ignore_yaw, is_gps=True)

    def go_to_gps_point(self, waypoint: List[float],
                        speed: float, ignore_yaw: bool = True) -> None:
        """Drone go to gps point"""
        self.__go_to(waypoint[0], waypoint[1], waypoint[2],
                     speed, ignore_yaw, is_gps=True)

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
