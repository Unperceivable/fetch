#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('ardrone_autonomy')
roslib.load_manifest('gazebo')

import message_filters
import rosbag
from math import pi
import cv2
import numpy as np
import matplotlib.pyplot as plt

#Quaternion to Euler
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_matrix, quaternion_from_euler

#import HOG class
from hogv0 import HOG

# Convert simulator messages into open cv / numpy array
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Pose,Twist

# 
from gazebo.msg import ModelStates

# Receiving the video feed
from sensor_msgs.msg import Image ,CameraInfo

# An enumeration of Drone Statuses
from drone_status import DroneStatus


class DroneCamera():
	def __init__(self):

		self.subInfo = rospy.Subscriber("/ardrone/front/camera_info", CameraInfo, self.CameraInfo)

		self.subImage = rospy.Subscriber("/ardrone/front/image_raw", Image,self.ReceiveImage)
		self.subPose = rospy.Subscriber("/gazebo/model_states", ModelStates,self.PoseInfo)
		
		self.objectf = open("/home/constantin/ros/ardrone_autonomy/bin/output/objectcoords.txt","w")
		self.envf= open("/home/constantin/ros/ardrone_autonomy/bin/output/envcoords.txt","w")


		self.videoFeed=False
		self.startHOG=False
		self.displayHOG=False
		self.compareHist=False
		
		self.ec=0
		self.oc=0
		
		self.hog=HOG((64,64),
			(37,32),
			(32,32),
			(16,16),
			(16,16),
			16,
			1,
			1)
		self.bridge = CvBridge()

		self.modelKey=-1		

		self.savedHist=np.zeros((1,1))
		self.currentHist=np.zeros((1,1))

		self.results=list()
		self.i=-1
		
	
	def PoseInfo(self,msg):
		if self.modelKey == -1:
			self.modelKey=msg.name.index("quadrotor")
			self.dronePose=Pose() 
			self.objectPose=Pose()
			self.objectPose.position.x = msg.pose[msg.name.index("drill_model")].position.x
			self.objectPose.position.y = msg.pose[msg.name.index("drill_model")].position.y
			self.objectPose.position.z = msg.pose[msg.name.index("drill_model")].position.z

		#Cartesian Pose
		self.dronePose.position.x = msg.pose[self.modelKey].position.x
		self.dronePose.position.y = msg.pose[self.modelKey].position.y
		self.dronePose.position.z = msg.pose[self.modelKey].position.z
		#Quaternion Orientation
		self.dronePose.orientation.x = msg.pose[self.modelKey].orientation.x
		self.dronePose.orientation.y = msg.pose[self.modelKey].orientation.y
		self.dronePose.orientation.z = msg.pose[self.modelKey].orientation.z
		self.dronePose.orientation.w = msg.pose[self.modelKey].orientation.w
		

	def CameraInfo(self,msg):
		info=np.asarray(msg.P)
		self.cameraInfo= np.asmatrix(info.reshape((3,4)))


 	# Save and process the ros image depending on flags
	def ReceiveImage(self,data):

		if self.videoFeed==True:
		
			try:
				cvImage = self.bridge.imgmsg_to_cv(data, "bgr8")			
				npImage = np.array(cvImage, dtype=np.uint8)
			except CvBridgeError, e:				
				print e


			if self.startHOG==True:
				#Compute HOG of Image
				sizedImage = cv2.resize(npImage,(256,128))
				grayImage = cv2.cvtColor(sizedImage, cv2.COLOR_RGB2GRAY)
				
				#Compute HOG of Cells and visual HOG
				if self.displayHOG:
					self.currentHist=self.hog.ComputeHOGCells(grayImage)
					hogImage=self.hog.visualizeHOG(grayImage,self.currentHist)
					cv2.imshow("HOG Image",hogImage)
				
				#Compare currentImage HOG to savedImage HOG
				if self.compareHist:
					if len(self.savedHist)==1:
						self.savedHist = self.hog.ComputeHOGCells(grayImage)
					self.currentHist = self.hog.ComputeHOGCells(grayImage)
					#print self.hog.CompareHOG(self.savedHist,self.currentHist)
					self.results.append(abs(sum(self.savedHist)-sum(self.currentHist)))
					self.i+=1
					if self.i%5==0:
						cv2.imwrite("/home/constantin/"+str(self.i)+".png",npImage)

				self.getPose()

				#print "Window: "+str((winY,winX))					
				#print (startY,endY,startX,endX)

				pos=np.matrix([[self.objectPose.position.x],[self.objectPose.position.y],[self.objectPose.position.z],[1]])		
				#pos=np.matrix([[1],[0],[1],[1]])
				#print self.angles
				# P*[R -Rd] *o[x y z 1]'
				#   [0  1 ]
				#print "c"
				c=np.matrix([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]])
				#print c			
				#print "Camera Projection p"			
				#print self.cameraInfo
				#print "Hx"
				#print self.transformationMatrix*pos
				#print "pc"
				pc = np.asmatrix(self.cameraInfo)*c
				#print pc
				p=np.asmatrix(self.cameraInfo)*c*self.transformationMatrix*pos
				#print "pcHx"			
				#print p
				x=p[0]/p[2]
				y=p[1]/p[2]

				if (0<=y and y<=npImage.shape[0]) and (0<=x and x<=npImage.shape[1]) and p[2]>0:
							cv2.circle(npImage, (x,y), 10, (0, 255, 0), -1)	
							self.objectf.write(str((float(x),float(y)))+'\n')
							cv2.imwrite("/home/constantin/ros/ardrone_autonomy/bin/output/raw data/object/"+str(self.oc)+".png",npImage)					
							self.oc+=1

							
				else:
							'self.envf.write(str((float(x),float(y)))+'\n')
							'cv2.imwrite("/home/constantin/ros/ardrone_autonomy/bin/output/raw data/environment/"+str(self.ec)+".png",npImage)
							'self.ec+=1
				
				#self.GenerateDescriptor(npImage)



			
				cv2.putText(npImage,	"dXpos: %.2f" % self.dronePose.position.x,(0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
				cv2.putText(npImage,	"dYpos: %.2f" % self.dronePose.position.y,(0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
				cv2.putText(npImage,	"dZpos: %.2f" % self.dronePose.position.z,(0,45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
			

				cv2.putText(npImage,	"oXpos: %.2f" % x,(0,60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
				cv2.putText(npImage,	"oYpos: %.2f" % y,(0,75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))

				cv2.putText(npImage,	"Roll: %f" % float(180*self.angles[0]/pi),(0,90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
				cv2.putText(npImage,	"Pitch: %f" % float(180*self.angles[1]/pi),(0,105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
				cv2.putText(npImage,	"Yaw: %f" % float(180*self.angles[2]/pi),(0,120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))

			cv2.imshow("Ardrone Front Camera", npImage)
			cv2.waitKey(10)
				

	def StartFeed(self):
		self.videoFeed=True
		cv2.namedWindow("Ardrone Front Camera",1)
	
	def EndFeed(self):
		self.videoFeed=False		
		cv2.destroyAllWindows()
	
	def StartHOG(self,
			winSize=(64,64),
			winStride=(37,32),
			blockSize=(32,32),
			blockStride=(16,16),
			cellSize=(16,16),
			nbins=16,
			scaleFactor=1,
			vizFactor=1):
		
		self.startHOG=True
		self.hog=HOG(winSize,
			winStride,
			blockSize,
			blockStride,
			cellSize,
			nbins,
			scaleFactor,
			vizFactor)

	def GenerateDescriptor(self,image,scaleCount=2):
		#2d projection using 3d point and projection matrix

		imageHeight=image.shape[0]	
		imageWidth=image.shape[1]
		for sc in xrange(scaleCount+1):
		
			sizedImage = cv2.resize(image,(int(imageWidth*pow(self.hog.scaleFactor,sc)),int(imageHeight*pow(self.hog.scaleFactor,sc))))
			imageHeight=sizedImage.shape[0]	
			imageWidth=sizedImage.shape[1]
			#print (imageWidth,imageHeight)
			#Check that the Window Parameters match the image Dimensions else raise an error quit
			if (imageWidth>=self.hog.winWidth) and ((imageWidth-self.hog.winWidth) % self.hog.winStrideX==0) and (imageHeight>=self.hog.winHeight) and ((imageHeight-self.hog.winHeight) % self.hog.winStrideY==0):	
				for winY in xrange((imageHeight-self.hog.winHeight)/self.hog.winStrideY+1):
					for winX in xrange((imageWidth-self.hog.winWidth)/self.hog.winStrideX+1):
					
						startY= winY*self.hog.winStrideY
						endY= winY*self.hog.winStrideY +  self.hog.winHeight
						startX = winX*self.hog.winStrideX
						endX= winX*self.hog.winStrideX +  self.hog.winWidth
						#print "Window: "+str((winY,winX))					
						#print (startY,endY,startX,endX)

						pos=np.matrix([[self.objectPose.position.x],[self.objectPose.position.y],[self.objectPose.position.z],[1]])		
						#pos=np.matrix([[1],[0],[1],[1]])
						#print self.angles
						# P*[R -Rd] *o[x y z 1]'
						#   [0  1 ]
						#print "c"
						c=np.matrix([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]])
						#print c			
						#print "Camera Projection p"			
						#print self.cameraInfo
						#print "Hx"
						#print self.transformationMatrix*pos
						#print "pc"
						pc = np.asmatrix(self.cameraInfo)*c
						#print pc
						p=np.asmatrix(self.cameraInfo)*c*self.transformationMatrix*pos
						#print "pcHx"			
						#print p
						x=p[0]/p[2]
						y=p[1]/p[2]
						self.x=x
						self.y=y
						if (startY<=y and y<=endY) and (startX<=x and x<=endX) and p[2]>0:
							self.objectf.write(str((x,y))+'\n')
							cv2.imwrite("/home/constantin/ros/ardrone_autonomy/bin/output/raw data/object/"+str(self.oc)+".png",image)					
							self.oc+=1	

						'''else:
							if(rand()>0.9):
								#self.envf.write(str(self.hog.computeBlocks(image[startY:endY,startX:endX]))+'\n')
								self.objectf.write(str((x,y))+'\n')
								cv2.imwrite("/home/constantin/ros/ardrone_autonomy/bin/output/raw data/environment/"+str(self.ec)+".png",image)
							self.ec+=1'''
			else:
				#raise CustomException('image Shape - Window Parameter mismatch: Window Parameters adhere to shape values: following conditional statements are in chcked order:')
				print (imageWidth>=self.hog.winWidth)
				print ((imageWidth-self.hog.winWidth) % self.hog.winStrideX==0)
				print (imageHeight>=self.hog.winHeight)
				print ((imageHeight-self.hog.winHeight)% self.hog.winStrideY)			
				return None



			

	def SaveHist(self):	
		self.compareHist = True

	def DrawHOG(self):
		self.displayHOG = True
		cv2.namedWindow("HOG Image",1)

	def ShowResults(self):
	
		print len(self.results)

		frames = list()
		for i in xrange(len(self.results)):
			frames.append(i)
		print len(frames)
		print frames[::5]
		print self.results[::5]
		plt.plot(np.array(frames[::5]),np.array(self.results[::5]))
		plt.savefig("/home/constantin/fig1.pdf")

    	def getPose(self):

            #Ros service call to get model state
            #This returns a GetModelStateResponse, which contains data on pose
            #rospy.wait_for_service('/gazebo/get_model_state')

            #  Use the tf module transforming quaternions to euler
            #try:
		self.angles = list(euler_from_quaternion([self.dronePose.orientation.x, self.dronePose.orientation.y, self.dronePose.orientation.z, self.dronePose.orientation.w]))
		
		#self.transformationMatrix=euler_matrix(self.angles[0]+pi, self.angles[1]+pi, self.angles[2]+pi, 'sxyz')
		
		#quaternion_from_euler(self.angles[0]+pi, self.angles[1]+pi, self.angles[2]+pi, 'szyx')

		# T=[R -Rd]	d[x y z]'
		#   [0  1 ]
	
		#Homogeneous tranformation matrix
		#HT = [R 0]
		#     [0 1]		
		self.transformationMatrix=quaternion_matrix([self.dronePose.orientation.x, self.dronePose.orientation.y, self.dronePose.orientation.z, self.dronePose.orientation.w]).transpose()
		#print "Homogeneous T"
		#print self.transformationMatrix
		#d[x y z]'
		d=np.matrix([[self.dronePose.position.x], [self.dronePose.position.y], [self.dronePose.position.z]])
		#print "Drone pos"		
		#print d

		#Rd= R*d
		Rd= self.transformationMatrix[:3,:3]*d

		# T=[R -Rd]
		#   [0  1 ]
		self.transformationMatrix[0,3]=-Rd[0,0]
		self.transformationMatrix[1,3]=-Rd[1,0]
		self.transformationMatrix[2,3]=-Rd[2,0]

		'''
		self.transformationMatrix[0,3]=d[0,0]
		self.transformationMatrix[1,3]=d[1,0]
		self.transformationMatrix[2,3]=d[2,0]
		 '''
		#print "T"
		#print self.transformationMatrix
            #except Exception:
                #print 'Pose Broke', Exception

'''
raw_img = cv2.imread('/home/constantin/Pictures/Screenshot from 2014-11-28 13:31:05.png',1)
raw_img = cv2.resize(raw_img,(256,128))
img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2GRAY)	
camera=DroneCamera()
a=camera.GenerateDescriptor(img,1)
'''
