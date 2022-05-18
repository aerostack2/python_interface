from behaviour_actions.action_handler import ActionHandler

from rclpy.action import ActionClient

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from as2_msgs.msg import TrajectoryWaypoints
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from as2_msgs.srv import GeopathToPath
from as2_msgs.action import FollowPath

from dataclasses import dataclass
from typing import Any
from tools.utils import path_to_list

class SendFollowPath(ActionHandler):
    @dataclass
    class FollowPathData:
        path: Any
        speed: float = 1.0
        yaw_mode: TrajectoryWaypoints.yaw_mode = TrajectoryWaypoints.KEEP_YAW
        is_gps: bool = False
        
    def __init__(self, drone, path_data):
        self._action_client = ActionClient(drone, FollowPath, f'{drone.get_drone_id()}/FollowPathBehaviour')
        self._drone = drone

        goal_msg = FollowPath.Goal()
        goal_msg.trajectory_waypoints = self.get_traj(path_data)

        try:
            super().__init__(self._action_client, goal_msg, drone.get_logger())
        except self.ActionNotAvailable as err:
            drone.get_logger().error(str(err))
        except (self.GoalRejected, self.GoalFailed) as err:
            drone.get_logger().warn(str(err))

    def get_traj(self, path_data):
        if isinstance(path_data.path, list):
            if not path_data.path:  # not empty
                raise Exception  # TODO
            if isinstance(path_data.path[0], list):
                point_list = path_data.path
            else:
                point_list = [path_data.path]
        elif isinstance(path_data.path, tuple):
            point_list = [list(path_data.path)]
        elif isinstance(path_data.path, Path):
            point_list = path_to_list(path_data.path)
            is_gps = False
        elif isinstance(path_data.path, GeoPath):
            req = GeopathToPath.Request()
            req.geo_path = path_data.path
            resp = self._drone.global_to_local_cli_.call(req)
            if not resp.success:
                self._drone.get_logger().warn("Can't follow path since origin is not set")
                raise Exception  # TODO

            point_list = path_to_list(resp.path)
            is_gps = False
        elif isinstance(path_data.path, TrajectoryWaypoints):
            return path_data.path
        else:
            raise Exception  # TODO
        
        if path_data.is_gps:
            geopath = GeoPath()
            geopath.header.stamp = self._drone.get_clock().now().to_msg()
            geopath.header.frame_id = "wgs84"
            for wp in point_list:
                gps = GeoPoseStamped()
                gps.header.stamp = geopath.header.stamp
                gps.header.frame_id = geopath.header.frame_id
                gps.pose.position.latitude = float(wp[0])
                gps.pose.position.longitude = float(wp[1])
                gps.pose.position.altitude = float(wp[2])
                geopath.poses.append(gps)
        
            path_data.path = geopath
            path_data.is_gps = False
            return self.get_traj(path_data)
        else:
            msg = TrajectoryWaypoints()
            msg.header.stamp = self._drone.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.yaw_mode = path_data.yaw_mode
            poses = []
            for point in point_list:
                pose = PoseStamped()
                x,y,z = point
                pose.pose.position.x = (float)(x)
                pose.pose.position.y = (float)(y)
                pose.pose.position.z = (float)(z)
                pose.pose.orientation.w=1.0
                poses.append(pose)
            msg.poses = poses
            msg.max_speed = (float)(path_data.speed)
            return msg
