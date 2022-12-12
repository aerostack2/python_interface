"""
motion_reference_handler_module.py
"""
from std_srvs.srv import SetBool

from motion_reference_handlers.hover_motion import HoverMotion
from motion_reference_handlers.position_motion import PositionMotion
from motion_reference_handlers.speed_motion import SpeedMotion
from motion_reference_handlers.speed_in_a_plane import SpeedInAPlaneMotion


class MotionReferenceHandlerModule:
    """Motion Reference Handlers module"""
    __alias__ = "motion_ref_handler"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

        # FIXME: temporaly, manually stoping behaviors
        self.trajectory_gen_cli = self.__drone.create_client(
            SetBool, "traj_gen/run_node")
        if not self.trajectory_gen_cli.wait_for_service(timeout_sec=3):
            self.__drone.get_logger().warn("Trajectory generator service not found")
            self.trajectory_gen_cli = None

        self.__hover_motion_handler = HoverMotion(self)
        self.position = PositionMotion(self)
        self.speed = SpeedMotion(self)
        self.speed_in_a_plane = SpeedInAPlaneMotion(self)

    def hover(self) -> None:
        """Stop and hover current position.
        """
        if self.trajectory_gen_cli is not None:
            self.__drone.get_logger().info("Calling trajectory generator")
            req = SetBool.Request()
            req.data = False
            resp = self.trajectory_gen_cli.call(req)
            if not resp.success:
                self.__drone.get_logger().warn("Cannot stop trajectory generator")
        self.__hover_motion_handler.send_hover()
        self.__drone.get_logger().info("Hover sent")

    def destroy(self) -> None:
        """Destroy module, clean exit"""
        self.__drone.destroy_client(self.trajectory_gen_cli)
