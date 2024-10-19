#!/usr/bin/env python		
import roslib; roslib.load_manifest('ardrone_autonomy')
import rospy
import time
import rosbag			#*Record Library?*
import subprocess
import cv2
import math

# Load the DroneController class, which handles interactions with the drone
from drone_controller import DroneController
from drone_camera_alt import DroneCamera

# Setup the application
if __name__=='__main__':
    import sys
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('ai')

    THRESHOLD=0.98

    #Initiate rotation/probability mapping mode
    SEARCH=True

    #Approach evaluated best target
    APPROACH=False

    DONE=False

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

    #drone.SetCommand(0,0,0,0.1)
    time.sleep(2)
    drone.SetCommand()

    gridResolution = camera.Map.gridResolution
    voxelResolution = camera.Map.voxelResolution

    gridDimensions=(camera.Map.mapWidth/gridResolution,camera.Map.mapLength/gridResolution,3)

    while not DONE:

        if SEARCH:
            for i in xrange(360/12):
                drone.Turn(12,0.3)
                time.sleep(2)

            SEARCH=False
            APPROACH=True


        if APPROACH:

            start=(float(camera.Map.dronePose[1]),float(camera.Map.dronePose[0]),float(camera.Map.dronePose[2]))
            rot=drone.GetrotZ()

            endcell = camera.Map.evaluate()
            end=( ( endcell[0]*gridResolution) + (gridResolution/2) , ( endcell[1]*gridResolution) + (gridResolution/2),( endcell[2]*voxelResolution) + (voxelResolution/2) )

            vector = (end[0]-start[0] ,end[1]-start[1] ,end[2]-start[2])

            print "3D Array Position: " +str(endcell)
            print "Drone Start Position: " +str(start)
            print "Drone End Position: " +str(end)


            #arctan(y/x) =  vector angle between two points
            rotation= 180*(math.atan(vector[1]/vector[0])/math.pi)

            #determin global rotation required

            #if x direction is greater than zero (right quadrents)
            if vector[0]>0:

                rotation=90-rotation


            #else left quadrents
            else:

                rotation=270-rotation

            drone.TurnTo(rotation)
            speed=0.2
            xdist=math.sqrt(vector[0]**2+vector[1]**2)-0.25
            zdist=abs(vector[2])-0.25

            print "Xdist Change Needed" + str(xdist)
            print "Zdist Change Needed" + str(zdist)

            print "Absolute Rotation: " + str(rotation)


            while  xdist>0 or zdist>0:

                if xdist>0:
                    pitch=speed
                else:
                    pitch=0

                if zdist>0:
                    if vector[2]>0:
                        zvel=speed
                    else:
                        zvel=-speed

                else:
                    zvel=0

                drone.SetCommand(0,pitch,0,zvel)
                time.sleep(1)

                newstart=camera.Map.dronePose

                xdist-=abs(start[0]-newstart[0])
                zdist-=abs(start[2]-newstart[2])
                start=newstart

            drone.SetCommand()
            print "**********"
            print xdist
            print zdist
            print "**********"

            if camera.Map[endcell[0]][endcell[1]][endcell[2]] >= THRESHOLD:
                camera.grabFrame()
                while camera.grab:
                    time.sleep(0)

                image=camera.frame
                hogData=[]
                imageHeight = image.shape[0]
                imageWidth = image.shape[1]

                minStartX=imageWidth
                minStartY=imageHeight
                maxEndX=0
                maxEndY=0

                #print (imageWidth,imageHeight)
                #Check that the Window Parameters match the image Dimensions else raise an error quit
                if (imageWidth >= camera.hog.winWidth) and ((imageWidth - camera.hog.winWidth) % camera.hog.winStrideX == 0) and (imageHeight >= camera.hog.winHeight) and ((imageHeight - camera.hog.winHeight) % camera.hog.winStrideY == 0):
                    for winY in xrange((imageHeight - camera.hog.winHeight) / camera.hog.winStrideY + 1):
                        for winX in xrange((imageWidth - camera.hog.winWidth) / camera.hog.winStrideX + 1):

                            startY = winY * camera.hog.winStrideY
                            endY = winY * camera.hog.winStrideY + camera.hog.winHeight
                            startX = winX * camera.hog.winStrideX
                            endX = winX * camera.hog.winStrideX + camera.hog.winWidth

                            hog=camera.hog.computeBlocks(image[startY:endY,startX:endX])
                            ret, results, neighbours, dist = camera.knn.find_nearest(hog, 3)

                            if results:
                                DONE=True
                                if startY<minStartY:
                                    minStartY=startY
                                if startX<minStartX:
                                    minStartX=startX
                                if endX>maxEndX:
                                    maxEndX=endX
                                if endY>maxEndY:
                                    maxEndY=endY
                else:
                    #raise CustomException('image Shape - Window Parameter mismatch: Window Parameters adhere to shape values: following conditional statements are in chcked order:')
                    print (imageWidth >= camera.hog.winWidth)
                    print ((imageWidth - camera.hog.winWidth) % camera.hog.winStrideX == 0)
                    print (imageHeight >= camera.hog.winHeight)
                    print ((imageHeight - camera.hog.winHeight) % camera.hog.winStrideY)



                if DONE:

                        cv2.imshow("Results", image)
                        cv2.waitKey(10)


            else:
                APPROACH=False
                SEARCH=True

     


    #and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit()
