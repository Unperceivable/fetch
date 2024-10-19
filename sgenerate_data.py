#!/usr/bin/env python		
import roslib; roslib.load_manifest('ardrone_autonomy')
import rospy
import time
import rosbag			#*Record Library?*
import subprocess
import cv2

# Load the DroneController class, which handles interactions with the drone
from drone_controller import DroneController
from drone_camera_alt import DroneCamera

# Setup the application
if __name__=='__main__':
    import sys
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('ai')

    # Now we construct our drone objects
    camera = DroneCamera()
    drone = DroneController()

    #SetCommand(+roll-,+pitch-,+yaw_velocity-,+z_velocity-)
    time.sleep(15)
    camera.StartFeed()
    time.sleep(2)
    drone.Takeoff()
    time.sleep(5)
    camera.StartHOG()
    time.sleep(2)

    #distanceing from object
    drone.SetCommand(0,0.2,0,0.1)
    time.sleep(8)
    drone.SetCommand()

    #rotation to face objects
    drone.Turn(-180)
    time.sleep(3)

    #approach floor object ahead
    drone.SetCommand(0.15,0.2,0,-0.15)
    time.sleep(6)
    drone.SetCommand(0,0.2,0,-0.12)
    time.sleep(7)
    drone.SetCommand(0,0.2,0,0)
    time.sleep(6)
    drone.SetCommand()

    #turn to left object
    drone.Turn(90)
    drone.SetCommand(0,0.29,0,0)

    time.sleep(6)
    drone.SetCommand()
    #lift and rotate
    drone.Turn(-180)
    drone.SetCommand(0.5,-0.1,0,0.2)
    time.sleep(7)
    drone.SetCommand()


    #arch turn
    drone.SetCommand(-0.25,0,0.1,0)
    time.sleep(30)

    camera.EndFeed()
    time.sleep(1)
    camera.objectf.close()
    camera.envf.close()
    drone.Land()
    time.sleep(5)


    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit()
