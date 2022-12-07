from as2_msgs.msg import TrajectoryWaypoints
from nav_msgs.msg import Path
from python_interface.behaviour_actions.followpath_behaviour import SendFollowPath


class FollowPathModule:
    __alias__ = "follow_path"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

    # def __call__(self, speed: float = 0.5) -> None:
    #     """Drone follow_path"""
    #     SendFollowPath(self.__drone, float(speed))

    def __follow_path(self, path: Path, speed: float, yaw_mode: int, is_gps: bool = False) -> None:
        path_data = SendFollowPath.FollowPathData(
            path, speed, yaw_mode, is_gps)
        SendFollowPath(self, path_data)

    def follow_path(self, path: Path, speed: float = 1.0,
                    yaw_mode: int = TrajectoryWaypoints.KEEP_YAW) -> None:
        """Drone follow path"""
        self.__follow_path(path, speed, yaw_mode)

    def follow_gps_path(self, wp_path: Path, speed: float = 1.0,
                        yaw_mode: int = TrajectoryWaypoints.KEEP_YAW) -> None:
        """Drone follow gps path"""
        self.__follow_path(wp_path, speed, yaw_mode, is_gps=True)

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
