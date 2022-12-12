from time import sleep

import rclpy

from python_interface.drone_interface import DroneInterface

rclpy.init()

drone_interface = DroneInterface("drone_sim_0", verbose=True)

drone_interface.takeoff(3, 2)
print("Takeoff completed\n")
sleep(1)

drone_interface.goto(0, 5, 3, speed=1.0)
drone_interface.goto(5, 0, 2, speed=1.0)

drone_interface.follow_path([[5, 0, 3],
                             [5, 5, 3],
                             [0, 5, 3],
                             [0, 0, 3]], 5)

print("Path finished")

drone_interface.land(0.2)
drone_interface.shutdown()

print("Bye!")
