"""
A collection of utils to easily command drones with AeroStack2.
"""

# Copyright (c) 2022 Universidad Politécnica de Madrid
# All Rights Reserved
#
# Licensed under the BSD-3-Clause (the "License");
# you may not use this file except in compliance with the License.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__authors__ = "Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"
__version__ = "0.1.0"

import threading
from time import sleep
from typing import List, Dict, Union

import rclpy
import rclpy.signals
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.parameter import Parameter

from std_srvs.srv import SetBool
from as2_msgs.msg import PlatformInfo, AlertEvent
from geometry_msgs.msg import PoseStamped, TwistStamped

from motion_reference_handlers.hover_motion import HoverMotion
from motion_reference_handlers.position_motion import PositionMotion
from motion_reference_handlers.speed_motion import SpeedMotion
from motion_reference_handlers.speed_in_a_plane import SpeedInAPlaneMotion

from python_interface.shared_data.platform_info_data import PlatformInfoData
from python_interface.shared_data.pose_data import PoseData
from python_interface.shared_data.twist_data import TwistData

from python_interface.service_clients.arming import Arm, Disarm
from python_interface.service_clients.offboard import Offboard

from python_interface.tools.utils import euler_from_quaternion


STATE = ["DISARMED", "LANDED", "TAKING_OFF", "FLYING", "LANDING", "EMERGENCY"]
YAW_MODE = ["NONE", "YAW_ANGLE", "YAW_SPEED"]
CONTROL_MODE = ["UNSET", "HOVER", "POSITION", "SPEED", "SPEED_IN_A_PLANE",
                "ATTITUDE", "ACRO", "TRAJECTORY", "ACEL"]
REFERENCE_FRAME = ["UNDEFINED_FRAME", "LOCAL_ENU_FRAME",
                   "BODY_FLU_FRAME", "GLOBAL_ENU_FRAME"]


class DroneInterface(Node):
    """Drone interface node"""
    modules = {}

    def __init__(self, drone_id: str = "drone0", verbose: bool = False,
                 use_sim_time: bool = False) -> None:
        super().__init__(f'{drone_id}_interface', namespace=drone_id)

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.__executor = rclpy.executors.SingleThreadedExecutor()
        if verbose:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.__info = PlatformInfoData()
        self.pose = PoseData()
        self.twist = TwistData()

        self.namespace = drone_id
        print(f"Starting {self.drone_id}")

        self.info_sub = self.create_subscription(
            PlatformInfo, 'platform/info', self.info_callback, qos_profile_system_default)

        # State subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped, 'self_localization/pose', self.pose_callback, qos_profile_sensor_data)

        self.twist_sub = self.create_subscription(
            TwistStamped, 'self_localization/twist', self.twist_callback, qos_profile_sensor_data)

        self.trajectory_gen_cli = self.create_client(
            SetBool, "traj_gen/run_node")
        if not self.trajectory_gen_cli.wait_for_service(timeout_sec=3):
            self.get_logger().warn("Trajectory generator service not found")
            self.trajectory_gen_cli = None

        self.hover_motion_handler = HoverMotion(self)
        self.position_motion_handler = PositionMotion(self)
        self.speed_motion_handler = SpeedMotion(self)
        self.speed_in_a_plane_motion_handler = SpeedInAPlaneMotion(self)

        self.alert_pub = self.create_publisher(
            AlertEvent, "alert_event", qos_profile_system_default)

        # self.__executor.add_node(self)
        # self.__executor.spin()
        # self.__executor.shutdown()
        # rclpy.shutdown()

        self.keep_running = True
        self.__executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.auto_spin)
        self.spin_thread.start()

        sleep(0.5)
        self.get_logger().info(f'{self.drone_id} interface initialized')

    def __del__(self) -> None:
        self.shutdown()

    def load_module(self, pkg: str) -> None:
        """load module on drone"""
        import importlib
        module = importlib.import_module(pkg)
        target = [t for t in dir(module) if "Module" in t]
        class_ = getattr(module, str(*target))
        setattr(self, class_.__alias__, class_(self))

    @property
    def drone_id(self) -> str:
        """drone id / namespace getter"""
        return self.namespace

    def info_callback(self, msg: PlatformInfo) -> None:
        """platform info callback"""
        self.__info.data = [int(msg.connected), int(msg.armed), int(msg.offboard), msg.status.state,
                            msg.current_control_mode.yaw_mode, msg.current_control_mode.control_mode,
                            msg.current_control_mode.reference_frame]

    def __get_info(self) -> List[int]:
        return self.__info.data

    @property
    def info(self) -> Dict[str, Union[bool, str]]:
        """get drone info"""
        info = self.__get_info()
        return {"connected": bool(info[0]), "armed": bool(info[1]), "offboard": bool(info[2]),
                "state": STATE[info[3]], "yaw_mode": YAW_MODE[info[4]],
                "control_mode": CONTROL_MODE[info[5]], "reference_frame": REFERENCE_FRAME[info[6]]}

    def pose_callback(self, pose_msg: PoseStamped) -> None:
        """pose stamped callback"""
        self.pose.position = [pose_msg.pose.position.x,
                              pose_msg.pose.position.y,
                              pose_msg.pose.position.z]

        self.pose.orientation = [
            *euler_from_quaternion(
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w)]

    @property
    def position(self) -> List[float]:
        """drone position getter"""
        return self.pose.position

    @property
    def orientation(self) -> List[float]:
        """drone orientation getter"""
        return self.pose.orientation

    def twist_callback(self, twist_msg: TwistStamped) -> None:
        """twist stamped callback"""
        self.twist.twist = [twist_msg.twist.linear.x,
                            twist_msg.twist.linear.y,
                            twist_msg.twist.linear.z]

    @property
    def speed(self) -> List[float]:
        """drone speed getter"""
        return self.twist.twist

    def arm(self) -> None:
        """Drone arming"""
        sleep(0.1)
        Arm(self)

    def disarm(self) -> None:
        """Drone disarming"""
        Disarm(self)

    def offboard(self) -> None:
        """Drone set offboard"""
        Offboard(self)

    # TODO: replace with executor callbacks
    def auto_spin(self) -> None:
        """Drone intern spin"""
        while rclpy.ok() and self.keep_running:
            self.__executor.spin_once()
            sleep(0.05)

    def shutdown(self) -> None:
        """Shutdown properly"""
        self.keep_running = False
        self.destroy_subscription(self.info_sub)
        self.destroy_subscription(self.pose_sub)

        self.spin_thread.join()
        print("Clean exit")

    def send_hover(self) -> None:
        if self.trajectory_gen_cli is not None:
            self.get_logger().info("Calling trajectory generator")
            req = SetBool.Request()
            req.data = False
            resp = self.trajectory_gen_cli.call(req)
            if not resp.success:
                self.get_logger().warn("Cannot stop trajectory generator")
        self.hover_motion_handler.send_hover()
        self.get_logger().info("Hover sent")

    def send_emergency_land(self) -> None:
        """Set controller to hover mode. yYu will have to take it control manually"""
        msg = AlertEvent()
        msg.alert = AlertEvent.FORCE_LAND
        self.get_logger().info("Starting emergency landing")
        while True and rclpy.ok():
            self.alert_pub.publish(msg)
            sleep(0.01)

    def send_emergency_hover(self) -> None:
        """Set controller to hover mode. yYu will have to take it control manually"""
        msg = AlertEvent()
        msg.alert = AlertEvent.FORCE_HOVER
        self.get_logger().info("Starting emergency hover")
        while True and rclpy.ok():
            self.alert_pub.publish(msg)
            sleep(0.01)

    def send_emergency_land_to_aircraft(self) -> None:
        """Call platform emergency land"""
        msg = AlertEvent()
        msg.alert = AlertEvent.EMERGENCY_LAND
        self.get_logger().info("Starting emergency aircraft landing")
        while True and rclpy.ok():
            self.alert_pub.publish(msg)
            sleep(0.01)

    def send_emergency_hover_to_aircraft(self) -> None:
        """Call platform hover. BE CAREFUL, you will have to take it control manually!"""
        msg = AlertEvent()
        msg.alert = AlertEvent.EMERGENCY_HOVER
        self.get_logger().info("Starting emergency aircraft hover")
        while True and rclpy.ok():
            self.alert_pub.publish(msg)
            sleep(0.01)

    def send_emergency_killswitch_to_aircraft(self) -> None:
        """Call platform stop. BE CAREFUL, motors will stop!"""
        msg = AlertEvent()
        msg.alert = AlertEvent.KILL_SWITCH
        self.get_logger().info("Starting emergency aircraft killswitch")
        while True and rclpy.ok():
            self.alert_pub.publish(msg)
            sleep(0.01)
