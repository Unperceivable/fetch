#!/usr/bin/env python		
					
import roslib; roslib.load_manifest('ardrone_autonomy')
import rospy
import time
import rosbag			#*Record Library?*
import subprocess
import cv2

# Load the DroneController class, which handles interactions with the drone
from drone_controller import DroneController
from drone_camera import DroneCamera

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ai')
	
	# Now we construct our drone objects
	camera = DroneCamera()
	drone = DroneController()
	
	time.sleep(8)
	camera.StartFeed()
	time.sleep(2)
	drone.Takeoff()
	time.sleep(5)
	print drone.GetrotZ()
	drone.Turn(215)
	print drone.GetrotZ()
	time.sleep(5)
	camera.StartHOG()
	time.sleep(2)
	drone.SetCommand(0,1,0,0)
	#camera.SaveHist()
	#camera.DrawHOG()	
	time.sleep(2.3)
	drone.SetCommand(0,0,0,0)
	time.sleep(5)
	for i in xrange(10):
		drone.SetCommand(10,0,-1,0)
		time.sleep(2)
	print "im done"
	#camera.ShowResults()
	camera.EndFeed()
	drone.Land()
	time.sleep(5)
	

	# executes the QT application
	#status = app.exec_()
	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit()
