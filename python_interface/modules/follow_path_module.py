"""
follow_path_module.py
"""

import typing

from as2_msgs.msg import TrajectoryWaypoints
from nav_msgs.msg import Path
from python_interface.behaviour_actions.followpath_behaviour import SendFollowPath

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class FollowPathModule:
    """Follow Path Module
    """
    __alias__ = "follow_path"

    def __init__(self, drone: 'DroneInterface') -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

        self.__current_fp = None

    def __follow_path(self, path: Path, speed: float,
                      yaw_mode: int, wait_result: bool = True) -> None:
        path_data = SendFollowPath.FollowPathData(
            path, speed, yaw_mode, is_gps=False)
        self.__current_fp = SendFollowPath(self, path_data, wait_result)

    def __call__(self, path: Path, speed: float,
                 yaw_mode: int = TrajectoryWaypoints.KEEP_YAW, wait: bool = True) -> None:
        """Follow path with speed (m/s) and yaw_mode.

        :type path: Path
        :type speed: float
        :param yaw_mode: yaw_mode, defaults to TrajectoryWaypoints.KEEP_YAW
        :type yaw_mode: int, optional
        """
        self.__follow_path(path, speed, yaw_mode, wait)

    # TODO
    def __del__(self):
        del self.__drone.modules[self.__alias__]

    def pause(self) -> None:
        # self.__current_fp.pause()
        # super().pause()  # Best way to do it. Take advantage of inheritance or multi-inheritance
        raise NotImplementedError

    def resume(self) -> None:
        # self.__current_fp.resume()
        raise NotImplementedError

    def stop(self) -> None:
        if self.__current_fp:
            self.__current_fp.stop()

    def modify(self, path: Path, speed: float,
               yaw_mode: int = TrajectoryWaypoints.KEEP_YAW) -> None:
        # path_data = SendFollowPath.FollowPathData(path, speed, yaw_mode, is_gps=False)
        # # path_data to goal_msg
        # self.__current_fp.modify(goal_msg=msg)
        raise NotImplementedError
