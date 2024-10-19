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

# Quaternion to Euler
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_matrix, quaternion_from_euler

#import HOG class
from hogv0 import HOG

from drone_map import DroneMap

# Convert simulator messages into open cv / numpy array
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseArray, Pose, Twist

# 
from gazebo.msg import ModelStates

# Receiving the video feed
from sensor_msgs.msg import Image, CameraInfo

# An enumeration of Drone Statuses
from drone_status import DroneStatus


class DroneCamera():
    def __init__(self):

        self.subInfo = rospy.Subscriber("/ardrone/front/camera_info", CameraInfo, self.CameraInfo)

        #self.subImage = rospy.Subscriber("/ardrone/front/image_raw", Image,self.ReceiveImage)
        self.subStates = rospy.Subscriber("/gazebo/model_states", ModelStates, self.PoseInfo)
        self.pubPose = rospy.Publisher("/stampedPoseArray", PoseArray)

        self.subImage = message_filters.Subscriber("/ardrone/front/image_raw", Image)
        self.subPose = message_filters.Subscriber("/stampedPoseArray", PoseArray)

        self.ts = message_filters.TimeSynchronizer([self.subImage, self.subPose], 10)
        self.ts.registerCallback(self.ReceiveImage)

        self.objectf = open("/home/constantin/ros/ardrone_autonomy/bin/output/objectcoords.txt", "w")
        self.envf = open("/home/constantin/ros/ardrone_autonomy/bin/output/envcoords.txt", "w")


        self.videoFeed = False
        self.startHOG = False
        self.displayHOG = False
        self.compareHist = False
        self.grab = False
        self.c = np.matrix([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])

        self.ec = 0
        self.oc = 0

        self.hog = HOG((64, 64),
                       (37, 32),
                       (32, 32),
                       (16, 16),
                       (16, 16),
                       16,
                       1,
                       1)
        self.bridge = CvBridge()

        self.savedHist = np.zeros((1, 1))
        self.currentHist = np.zeros((1, 1))

        self.results = list()
        self.i = -1
        self.droneIndex = -1
        self.poseArray = PoseArray()

        self.Map=DroneMap()
        self.Map.Train()

    def PoseInfo(self, msg):

        #Initialize PoseArray with Drone and Object Poses
        if self.droneIndex == -1:
            self.droneIndex = msg.name.index("quadrotor")

            self.poseArray.header.frame_id = "/base_link"
            self.poseArray.header.stamp = rospy.Time.now()

            self.poseArray.poses.append(msg.pose[self.droneIndex])

            '''self.poseArray.poses.append(msg.pose[msg.name.index("bowl_model")])
            self.poseArray.poses.append(msg.pose[msg.name.index("bowl_model_0")])
            self.poseArray.poses.append(msg.pose[msg.name.index("bowl_model_1")])
            self.poseArray.poses.append(msg.pose[msg.name.index("bowl_model_2")])
            self.poseArray.poses.append(msg.pose[msg.name.index("bowl_model_3")])
            self.poseArray.poses.append(msg.pose[msg.name.index("bowl_model_4")])
            self.poseArray.poses.append(msg.pose[msg.name.index("bowl_model_5")])
            self.poseArray.poses.append(msg.pose[msg.name.index("bowl_model_6")])'''

            self.poseArray.poses.append(msg.pose[msg.name.index("drill_model")])
            self.poseArray.poses.append(msg.pose[msg.name.index("drill_model_0")])
            self.poseArray.poses.append(msg.pose[msg.name.index("drill_model_1")])
            self.poseArray.poses.append(msg.pose[msg.name.index("drill_model_2")])
            self.poseArray.poses.append(msg.pose[msg.name.index("drill_model_3")])
            self.poseArray.poses.append(msg.pose[msg.name.index("drill_model_4")])
            self.poseArray.poses.append(msg.pose[msg.name.index("drill_model_5")])
            self.poseArray.poses.append(msg.pose[msg.name.index("drill_model_6")])

            for n in self.poseArray.poses[1:]:
                #n.position.x+=0.0185
                #n.position.y+=0.050
                #n.position.z+=0.25
                n.position.z += 0.0
                self.pubPose.publish(self.poseArray)


        #Add header to poseArray
        else:
            self.poseArray.header.frame_id = "/base_link"
            self.poseArray.header.stamp = rospy.Time.now()

            #Update Stamped Drone Pose

            #Cartesian Pose
            self.poseArray.poses[0] = msg.pose[self.droneIndex]
            '''self.poseArray.poses[0].position.y = msg.pose[self.droneIndex].position.y
            self.poseArray.poses[0].position.z = msg.pose[self.droneIndex].position.z
            #Quaternion Orientation
            self.poseArray.poses[0].orientation.x = msg.pose[self.droneIndex].orientation.x
            self.poseArray.poses[0].orientation.y = msg.pose[self.droneIndex].orientation.y
            self.poseArray.poses[0].orientation.z = msg.pose[self.droneIndex].orientation.z
            self.poseArray.poses[0].orientation.w = msg.pose[self.droneIndex].orientation.w'''

            self.pubPose.publish(self.poseArray)
        #rospy.sleep(1)


    def CameraInfo(self, msg):
        info = np.asarray(msg.P)
        self.cameraInfo = np.asmatrix(info.reshape((3, 4)))


    # Save and process the ros image depending on flags
    def ReceiveImage(self, image, poseArray):
        if self.videoFeed == True:

            try:
                cvImage = self.bridge.imgmsg_to_cv(image, "bgr8")
                npImage = np.array(cvImage, dtype=np.uint8)

                if self.startHOG == True:
                    #Compute HOG of Image
                    sizedImage = cv2.resize(npImage, (256, 128))
                    grayImage = cv2.cvtColor(sizedImage, cv2.COLOR_RGB2GRAY)

                    #Compute HOG of Cells and visual HOG
                    if self.displayHOG:
                        self.currentHist = self.hog.ComputeHOGCells(grayImage)
                        hogImage = self.hog.visualizeHOG(grayImage, self.currentHist)
                        cv2.imshow("HOG Image", hogImage)

                    #Compare currentImage HOG to savedImage HOG
                    if self.compareHist:
                        if len(self.savedHist) == 1:
                            self.savedHist = self.hog.ComputeHOGCells(grayImage)
                        self.currentHist = self.hog.ComputeHOGCells(grayImage)
                        #print self.hog.CompareHOG(self.savedHist,self.currentHist)
                        self.results.append(abs(sum(self.savedHist) - sum(self.currentHist)))
                        self.i += 1
                        if self.i % 5 == 0:
                            cv2.imwrite("/home/constantin/" + str(self.i) + ".png", npImage)


                    #When grab Frame is requested
                    #if True: #self.grab:
                    #    self.grab=False
                    self.getPose(poseArray)

                    if self.grab:
                        self.frame=np.Image
                        self.grab=False

                    self.Map.Update(npImage,self.cameraInfo,np.asmatrix(self.rotationMatrix),np.asmatrix(self.dronePose),self.c[:3, :3])
                    #print "Wolrd Coord: " + str((float(self.dronePose[0]),float(self.dronePose[1]),float(self.dronePose[2])))

                    #Create List to store drone position and object positions in enviorment at time of frame
                    imageDetails = list()
                    imageDetails.append((float(poseArray.poses[0].position.x), float(poseArray.poses[0].position.y),
                                         float(poseArray.poses[0].position.z), float(poseArray.poses[0].orientation.x),
                                         float(poseArray.poses[0].orientation.y), float(poseArray.poses[0].orientation.z),
                                         float(poseArray.poses[0].orientation.w)))
                    flag = 0
                    for n in xrange(1, len(poseArray.poses)):
                        pos = np.matrix(
                            [[poseArray.poses[n].position.x], [poseArray.poses[n].position.y], [poseArray.poses[n].position.z],
                             [1]])


                        #pos=np.matrix([[1],[0],[1],[1]])
                        #print self.angles
                        # P*[R -Rd] *o[x y z 1]'Inspectah Deck, Triumph
                        #   [0  1 ]
                        #print "c"
                        #print c
                        #print "Camera Projection p"
                        #print self.cameraInfo
                        #print "Hx"
                        #print self.t#print "pc"
                        pc = np.asmatrix(self.cameraInfo) * self.c
                        #print pc
                        p = np.asmatrix(self.cameraInfo) * self.c * self.transformationMatrix * pos


                        #print "pcHx"
                        #print p
                        x = p[0] / p[2]
                        y = p[1] / p[2]
                        imageDetails.append((float(p[2]), float(x), float(y)))
                        if p[2] > 0 and (0 <= y and y <= npImage.shape[0]) and (0 <= x and x <= npImage.shape[1]):


                            if flag == 0:
                                #cv2.imwrite("/home/constantin/ros/ardrone_autonomy/bin/output/raw data/sample/object/" + str(self.oc) + ".png",npImage)
                                flag = 1

                            cv2.circle(npImage, (x, y), 5, (0, 255, 0), -1)

                            '''else:
                                                self.envf.write(str((float(x),float(y)))+'\n')
                                                cv2.imwrite("/home/constantin/ros/ardrone_autonomy/bin/output/raw data/environment/"+str(self.ec)+".png",npImage)
                                                self.ec+=1'''


                    if flag:
                        self.oc += 1
                        #self.objectf.write(str(imageDetails) + '\n')


                    #self.GenerateDescriptor(npImage,poseArray)

                    '''cv2.putText(npImage, "dXpos: %.2f" % poseArray.poses[0].position.x, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255))
                    cv2.putText(npImage, "dYpos: %.2f" % poseArray.poses[0].position.y, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255))
                    cv2.putText(npImage, "dZpos: %.2f" % poseArray.poses[0].position.z, (0, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255))

                    cv2.putText(npImage, "oXpos: %.2f" % x, (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
                    cv2.putText(npImage, "oYpos: %.2f" % y, (0, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

                    cv2.putText(npImage, "Roll: %f" % float(180 * self.angles[0] / pi), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255))
                    cv2.putText(npImage, "Pitch: %f" % float(180 * self.angles[1] / pi), (0, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255))
                    cv2.putText(npImage, "Yaw: %f" % float(180 * self.angles[2] / pi), (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 255))

                    cv2.imshow("Ardrone Front Camera", npImage)
                    cv2.waitKey(10)'''

            except CvBridgeError, e:
                print e


    def StartFeed(self):
        self.videoFeed = True
        cv2.namedWindow("Ardrone Front Camera", 1)


    def EndFeed(self):
        self.videoFeed = False
        cv2.destroyAllWindows()


    def StartHOG(self,
                 winSize=(64, 64),
                 winStride=(37, 32),
                 blockSize=(32, 32),
                 blockStride=(16, 16),
                 cellSize=(16, 16),
                 nbins=16,
                 scaleFactor=1,
                 vizFactor=1):
        self.startHOG = True
        self.hog = HOG(winSize,
                       winStride,
                       blockSize,
                       blockStride,
                       cellSize,
                       nbins,
                       scaleFactor,
                       vizFactor)


    def GenerateDescriptor(self, image, poseArray, scaleCount=2):
        #2d projection using 3d point and projection matrix

        imageHeight = image.shape[0]
        imageWidth = image.shape[1]
        for sc in xrange(scaleCount + 1):

            sizedImage = cv2.resize(image, (
                int(imageWidth * pow(self.hog.scaleFactor, sc)), int(imageHeight * pow(self.hog.scaleFactor, sc))))
            imageHeight = sizedImage.shape[0]
            imageWidth = sizedImage.shape[1]
            #print (imageWidth,imageHeight)
            #Check that the Window Parameters match the image Dimensions else raise an error quit
            if (imageWidth >= self.hog.winWidth) and ((imageWidth - self.hog.winWidth) % self.hog.winStrideX == 0) and (
                        imageHeight >= self.hog.winHeight) and ((imageHeight - self.hog.winHeight) % self.hog.winStrideY == 0):
                for winY in xrange((imageHeight - self.hog.winHeight) / self.hog.winStrideY + 1):
                    for winX in xrange((imageWidth - self.hog.winWidth) / self.hog.winStrideX + 1):

                        startY = winY * self.hog.winStrideY
                        endY = winY * self.hog.winStrideY + self.hog.winHeight
                        startX = winX * self.hog.winStrideX
                        endX = winX * self.hog.winStrideX + self.hog.winWidth
                        #print "Window: "+str((winY,winX))
                        #print (startY,endY,startX,endX)

                        pos = np.matrix([[poseArray.poses[1].position.x], [poseArray.poses[1].position.y],
                                         [poseArray.poses[1].position.z], [1]])
                        #pos=np.matrix([[1],[0],[1],[1]])
                        #print self.angles
                        # P*[R -Rd] *o[x y z 1]'
                        #   [0  1 ]
                        #print "c"
                        #print c
                        #print "Camera Projection p"
                        #print self.cameraInfo
                        #print "Hx"
                        #print self.transformationMatrix*pos
                        #print "pc"
                        pc = np.asmatrix(self.cameraInfo) * self.c
                        #print pc
                        p = np.asmatrix(self.cameraInfo) * self.c * self.transformationMatrix * pos
                        #print "pcHx"
                        #print p
                        x = p[0] / p[2]
                        y = p[1] / p[2]
                        self.x = x
                        self.y = y
                        if (startY <= y and y <= endY) and (startX <= x and x <= endX) and p[2] > 0:
                            self.objectf.write(str((x, y)) + '\n')
                            #cv2.imwrite("/home/constantin/ros/ardrone_autonomy/bin/output/raw data/object/" + str(self.oc) + ".png",image)
                            self.oc += 1

                        else:
                            #self.envf.write(str(self.hog.computeBlocks(image[startY:endY,startX:endX]))+'\n')
                            #cv2.imwrite("/home/constantin/ros/ardrone_autonomy/bin/output/raw data/environment/" + str(self.ec) + ".png", image)
                            self.ec += 1
            else:
                #raise CustomException('image Shape - Window Parameter mismatch: Window Parameters adhere to shape values: following conditional statements are in chcked order:')
                print (imageWidth >= self.hog.winWidth)
                print ((imageWidth - self.hog.winWidth) % self.hog.winStrideX == 0)
                print (imageHeight >= self.hog.winHeight)
                print ((imageHeight - self.hog.winHeight) % self.hog.winStrideY)
                return None


    def grabFrame(self):
        self.grab = True


    def SaveHist(self):
        self.compareHist = True


    def DrawHOG(self):
        self.displayHOG = True
        cv2.namedWindow("HOG Image", 1)


    def ShowResults(self):
        print len(self.results)

        frames = list()
        for i in xrange(len(self.results)):
            frames.append(i)
        print len(frames)
        print frames[::5]
        print self.results[::5]
        plt.plot(np.array(frames[::5]), np.array(self.results[::5]))
        plt.savefig("/home/constantin/fig1.pdf")


    def getPose(self, poseArray):
        #Ros service call to get model state
        #This returns a GetModelStateResponse, which contains data on pose
        #rospy.wait_for_service('/gazebo/get_model_state')

        #  Use the tf module transforming quaternions to euler
        #try:
        self.angles = list(euler_from_quaternion(
            [poseArray.poses[0].orientation.x, poseArray.poses[0].orientation.y, poseArray.poses[0].orientation.z,
             poseArray.poses[0].orientation.w]))

        #self.transformationMatrix=euler_matrix(self.angles[0]+pi, self.angles[1]+pi, self.angles[2]+pi, 'sxyz')

        #quaternion_from_euler(self.angles[0]+pi, self.angles[1]+pi, self.angles[2]+pi, 'szyx')

        # T=[R -Rd]	d[x y z]'
        #   [0  1 ]

        #Homogeneous tranformation matrix
        #HT = [R 0]
        #     [0 1]
        self.transformationMatrix = quaternion_matrix(
            [poseArray.poses[0].orientation.x, poseArray.poses[0].orientation.y, poseArray.poses[0].orientation.z,
             poseArray.poses[0].orientation.w]).transpose()
        self.rotationMatrix=self.transformationMatrix[:3, :3]
        #print "Homogeneous T"
        #print self.transformationMatrix
        #d[x y z]'
        d = np.matrix([[poseArray.poses[0].position.x], [poseArray.poses[0].position.y], [poseArray.poses[0].position.z]])
        #print "Drone pos"
        #print d
        self.dronePose=d.copy()

        #Rd= R*d
        Rd = self.transformationMatrix[:3, :3] * d

        # T=[R -Rd]
        #   [0  1 ]
        self.transformationMatrix[0, 3] = -Rd[0, 0]
        self.transformationMatrix[1, 3] = -Rd[1, 0]
        self.transformationMatrix[2, 3] = -Rd[2, 0]

        '''
            self.transformationMatrix[0,3]=d[0,0]
            self.transformationMatrix[1,3]=d[1,0]
            self.transformationMatrix[2,3]=d[2,0]
             '''

    def getFrame(self):

        self.grab=True


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
