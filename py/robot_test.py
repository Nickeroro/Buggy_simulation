import rob1a_v01 as rob1a  # get robot simulator
import robot_control  # get robot control functions 
import numpy as np
import time

if __name__ == "__main__":
    rb = rob1a.Rob1A()  # create a robot (instance of Rob1A class)
    ctrl = robot_control.RobotControl()  # create a robot controller

    rb.log_file_on()  # start log

    # angle = 90.0

    ctrl.goLineOdometer(rb, dist=0.25, speed=50)
    ctrl.inPlaceTurnRight(rb, 90.0)

    ctrl.goLineOdometer(rb, dist=0.25, speed=50)
    ctrl.inPlaceTurnLeft(rb, 90.0)

    ctrl.goLineOdometer(rb, dist=0.72, speed=50)
    ctrl.inPlaceTurnRight(rb, 100.0)

    ctrl.goLineOdometer(rb, dist=0.75, speed=50)
    ctrl.inPlaceTurnRight(rb, 90.0)

    ctrl.followWallsLeft(rb)
    ctrl.inPlaceTurnRight(rb, 95.0)

    ctrl.goLineOdometer(rb, dist=0.25, speed=50)
    ctrl.followWallsRight(rb)

    ctrl.inPlaceTurnRight(rb, 90.0)
    ctrl.goLineOdometer(rb, dist=0.25, speed=50)
    ctrl.followWallsRight(rb)

    ctrl.goLineOdometer(rb, dist=0.25, speed=50)

    # safe end : stop the robot, then stop the simulator
    rb.stop()
    rb.log_file_off()  # end log
    rb.full_end()
