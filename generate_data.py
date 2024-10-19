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
    time.sleep(8)
    camera.StartFeed()
    time.sleep(2)
    drone.Takeoff()
    time.sleep(5)
    camera.StartHOG()
    time.sleep(2)
    #far shot of central table objects
    drone.SetCommand(0.2,0.2,-0.25,0.2)
    time.sleep(2)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(3)

    #approach floor objects
    drone.SetCommand(0,0.35,0.4,-0.45)
    time.sleep(6)
    drone.SetCommand()
    #fine tune approach to left object
    drone.SetCommand(0.25,0.15,-0.1,0)
    time.sleep(4)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(2)

    #turn to second floor(right) object
    drone.Turn(-80)
    time.sleep(2)
    drone.SetCommand()
    time.sleep(1)
    #approach object
    drone.SetCommand(0,0.5,0,0)
    time.sleep(6)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(1)
    #turn to shelf object
    drone.Turn(-90)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(1)
    #approach self objecct with rising drift
    drone.SetCommand(-0.1,0.5,0.125,0.1)
    time.sleep(12)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(1)
    #turn to face and level with shelf object
    drone.SetCommand(-0.3,0,0.325,0.2)
    time.sleep(5)
    drone.SetCommand()
    camera.grabFrame()

    #spin with rising drift to record entire room
    drone.SetCommand(-0.6,-0.5,0.8,0)
    time.sleep(10)

    drone.SetCommand()
    camera.grabFrame()
    time.sleep(3)

    #turn to face furthest table drill
    drone.Turn(30)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(3)

    #approach furthest drill
    drone.SetCommand(0,0.5,0,0)
    time.sleep(6)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(1)

    #start S maneouver
    drone.SetCommand(0.35,0.7,-0.5,0)
    time.sleep(16)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(1)

    #complete S maneouver
    drone.SetCommand(-0.35,0.7,0.5,0)
    time.sleep(7)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(1)
    drone.SetCommand(0,0,0.3,-0.30)
    time.sleep(10)
    drone.SetCommand()
    camera.grabFrame()
    time.sleep(10)




    camera.EndFeed()
    time.sleep(1)
    camera.objectf.close()
    camera.envf.close()
    drone.Land()
    time.sleep(5)


    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit()
