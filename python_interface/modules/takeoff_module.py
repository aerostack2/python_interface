from as2_msgs.action import TakeOff

from python_interface.behaviour_actions.takeoff_behaviour import SendTakeoff


class TakeoffModule:
    __alias__ = "takeoff"

    def __init__(self, drone) -> None:
        self.__drone = drone
        self.__drone.modules[self.__alias__] = self

        self.__current_tk = None

    def __call__(self, height: float = 1.0, speed: float = 0.5, wait: bool = True) -> None:
        """Takeoff to given height (m) and given speed (m/s).

        :type height: float
        :type speed: float
        """
        self.__current_tk = SendTakeoff(self.__drone, float(height), float(speed))

    # TODO
    def __del__(self):
        del self.__drone.modules[self.__alias__]

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    def stop(self):
        if self.__current_tk:
            self.__current_tk.stop()

    def modify(self, height, speed):
        if self.__current_tk:
            goal_msg = TakeOff.Goal()
            goal_msg.takeoff_height = height
            goal_msg.takeoff_speed = speed
        raise NotImplementedError
