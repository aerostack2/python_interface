from as2_msgs.action import Land

from python_interface.behaviour_actions.land_behaviour import SendLand


class LandModule:
    __alias__ = "land"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

        self.__current_land = None

    def __call__(self, speed: float = 0.5, wait: bool = True) -> None:
        """Land with given speed (m/s).

        :type speed: float
        """
        self.__current_land = SendLand(self.__drone, float(speed))

    # TODO
    def __del__(self):
        del self.__drone.modules[self.__alias__]

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    def stop(self):
        if self.__current_land:
            self.__current_land.stop()

    def modify(self, speed):
        if self.__current_land:
            goal_msg = Land.Goal()
            goal_msg.land_speed = speed
            self.__current_land.modify(goal_msg)
