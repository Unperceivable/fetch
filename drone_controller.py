#!/usr/bin/env python 

#import system and utility libraries
import sys
import time
import math

#Import ROS libraries and load the manifest file linked to proj
import roslib; roslib.load_manifest('ardrone_autonomy')
import rospy			#*Needed to interface with ros?*
import rosbag			#*Record Library?*


from geometry_msgs.msg import Twist, Pose	#Required to send commands to drone
from  std_msgs.msg import Empty		#Needed for takeoff/land/emergency commands
from ardrone_autonomy.msg import Navdata	#Used to receive navadata feedback

#Enumeration of Drone States
from drone_status import DroneStatus


#Constant Declaration
COMMAND_PERIOD = 100 #ms




class DroneController(object):

    def __init__(self):
        #Record Drone's state
        self.status = -1
        self.rotZ = 0

        #Set Publish commands to /ardrone/ (takeoff, land, reset)
        self.pubLand = rospy.Publisher('/ardrone/land',Empty)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
        self.pubReset = rospy.Publisher('/ardrone/reset',Empty)
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

        #Subscribe to /ardrone/navdata topics to receive
        self.subNavdata = rospy.Subscriber('ardrone/navdata',Navdata,self.ReceiveNavdata)


        #Setup regular publishing of control packets
        self.command = Twist()

        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

        #Land drone on shutdown
        rospy.on_shutdown(self.Land)


    #Rotates drone by angle left(positive) amd right(negative)
    def Turn(self,rotation,speed=0.5):
        #phase = (phase + 180)%360 - 180 wraps phases from 180 to -180 and vice versa


        prevRot = self.GetrotZ()+180
        rot=0

        while rot<=abs(rotation):
            currRot=self.GetrotZ()+180
            rotDifference = abs(prevRot-currRot)


            if rotDifference > 300:
                Min=min(prevRot,currRot)
                Max=max(prevRot,currRot)
                rot+= (360-Max)+Min

            else:
                rot+=rotDifference

            prevRot = currRot

            #if turn left
            if rotation > 0:
                self.SetCommand(0,0,speed,0)

            # if turn right
            elif rotation < 0:
                self.SetCommand(0,0,-speed,0)

        self.SetCommand(0,0,0,0)

    def GetrotZ(self):
        return self.rotZ

    def ReceiveNavdata(self,navdata):
        #Interesting data in navdata packet
        self.status = navdata.state
        self.rotZ = navdata.rotZ

    def Takeoff(self):
        #Send takeoff message to ardrone driver
        #Check if landed before takeoff
        #if(self.status==DroneStatus.Landed):
        self.pubTakeoff.publish(Empty())
    def Land(self):
        self.pubLand.publish(Empty())

    def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
        #Set the current command
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity


    def SendCommand(self,event):
        #Previously set command is refreshed periodically if drone is flying
        if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
            self.pubCommand.publish(self.command)

    def TurnTo(self,angle,speed=0.5):

        curRot = self.GetrotZ()+180

        #calculate conventional turn distance
        normTurn=abs(curRot-angle)

        #calculate boundry turn distance
        boundTurn=min((curRot + (360-angle)),(angle+(360-curRot)))

        if normTurn < boundTurn:

            if curRot<angle:
                self.Turn(normTurn,speed)

            else:
                self.Turn(-normTurn,speed)

        else:
            if curRot<angle:
                self.Turn(-boundTurn,speed)

            else:
                self.Turn(boundTurn,speed)
