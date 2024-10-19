#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pyplot as plt
import random

#import HOG class
from hogv0 import HOG

from random import uniform
import glob
import re
# Receiving the video feed
from sensor_msgs.msg import Image


objectc = open("/home/constantin/ros/ardrone_autonomy/bin/output/objectcoords.txt","r+")

features = open("/home/constantin/ros/ardrone_autonomy/bin/output/features.txt","w")

#objectpath="/home/constantin/ros/ardrone_autonomy/bin/output/raw data/bowl/*.png"
#objectpath="/home/constantin/ros/ardrone_autonomy/bin/output/raw data/object/*.png"

#objectpath="/home/constantin/ros/ardrone_autonomy/bin/output/raw data/sample/bowl/*.png"
objectpath="/home/constantin/ros/ardrone_autonomy/bin/output/raw data/sample/object/*.png"


#envpath="/home/constantin/ros/ardrone_autonomy/bin/output/raw data/environment/*.png"
envpath="/home/constantin/ros/ardrone_autonomy/bin/output/raw data/sample/environment/*.png"

#objectfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/bowl/"
#objectfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/object/"
#objectfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/bowl/"
objectfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/object/"

#envfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/environment/"
envfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/environment/"


hog=HOG((64,64),(37,32),(32,32),(16,16),(16,16),16,0.5,1)

fileCounter=0

files=glob.glob(objectpath)

#sort files alphanumerically and not ASCIInumerically

def tryint(s):
    try:
        return int(s)
    except:
        return s
    
def alphanum_key(s):

    return [ tryint(c) for c in re.split('([0-9]+)', s) ]

files.sort(key=alphanum_key)

'''
for fname in files:


	imageDetails = eval(objectc.readline())
	image=cv2.imread(fname)
	scaleCount=2
	imageHeight=image.shape[0]	
	imageWidth=image.shape[1]

	#print (imageWidth,imageHeight)
	#for obj in imageDetails[1:]:
	#	print obj[1:]
	for sc in xrange(scaleCount+1):
			sizedImage = cv2.resize(image,(int(imageWidth*pow(hog.scaleFactor,sc)),int(imageHeight*pow(hog.scaleFactor,sc))))
			imageHeight=sizedImage.shape[0]	
			imageWidth=sizedImage.shape[1]
			#print (imageWidth,imageHeight)
			#Check that the Window Parameters match the image Dimensions else raise an error quit
			if (imageWidth>=hog.winWidth) and ((imageWidth-hog.winWidth) % hog.winStrideX==0) and (imageHeight>=hog.winHeight) and ((imageHeight-hog.winHeight) % hog.winStrideY==0):	
				for winY in xrange((imageHeight-hog.winHeight)/hog.winStrideY+1):
					for winX in xrange((imageWidth-hog.winWidth)/hog.winStrideX+1):
						#print "WinY: " + str(winY) + ", winX: " + str(winX)
						startY= winY*hog.winStrideY
						endY= winY*hog.winStrideY +  hog.winHeight
						startX = winX*hog.winStrideX
						endX= winX*hog.winStrideX +  hog.winWidth
						#print (startY,endY,startX,endX)
						flag=False
						objectCounter=0
						for obj in imageDetails[1:]:
							# object = (p[2], x, y)
							if (obj[0]>0) and (startY<=obj[2] and obj[2]<=endY) and (startX<= obj[1] and obj[1]<=endX):					
								flag=True								
								#features.write(str(hog.computeBlocks(image[startY:endY,startX:endX]))+'\n')								
								des=np.append([1],hog.computeBlocks(image[startY:endY,startX:endX]))
								
								np.savetxt(features,des[None], delimiter=',')

								#cv2.imwrite(objectfpath+str(fileCounter)+'-'+str((winY,winX))+'-'+str(objectCounter)+".png",image[startY:endY,startX:endX])
								#cpyimage=image
								#cv2.rectangle(cpyimage,(startX,startY),(endX,endY),50)
								#cv2.imshow("Ardrone Front Camera",cpyimage)
								#print str(fileCounter)+'-'+str((winY,winX))+'-'+str(objectCounter)
								#cv2.waitKey(0)

								#cv2.imshow("block", image[startY:endY,startX:endX])
								#cv2.waitKey(10)
	
							objectCounter+=1
							if (not flag) and random.random()>0.999:
								des=np.append([0],hog.computeBlocks(image[startY:endY,startX:endX]))
								np.savetxt(features,des[None], delimiter=',')
								cv2.imwrite(envfpath+str(fileCounter)+'-'+str((winY,winX))+'-'+str(objectCounter)+".png",image[startY:endY,startX:endX])
			else:
				#raise CustomException('image Shape - Window Parameter mismatch: Window Parameters adhere to shape values: following conditional statements are in chcked order:')
				print (imageWidth>=hog.winWidth)
				print ((imageWidth-hog.winWidth) % hog.winStrideX==0)
				print (imageHeight>=hog.winHeight)
				print ((imageHeight-hog.winHeight)% hog.winStrideY)			
				#return None
	fileCounter+=1'''

'''fileCounter=0

for fname in files:
    imageDetails = eval(objectc.readline())
    image=cv2.imread(fname)
    scaleCount=2
    imageHeight=image.shape[0]
    imageWidth=image.shape[1]

	#print (imageWidth,imageHeight
	#for obj in imageDetails[1:]:
	#print obj[1:]
    for sc in xrange(scaleCount):
        sizedImage = cv2.resize(image,(int(imageWidth*pow(hog.scaleFactor,sc)),int(imageHeight*pow(hog.scaleFactor,sc))))
        imageHeight=sizedImage.shape[0]
        imageWidth=sizedImage.shape[1]
        #print (imageWidth,imageHeight)
        #Check that the Window Parameters match the image Dimensions else raise an error quit
        if (imageWidth>=hog.winWidth)  and (imageHeight>=hog.winHeight):

            if not (((imageWidth-hog.winWidth) % hog.winStrideX==0) and ((imageHeight-hog.winHeight) % hog.winStrideY==0)):

                sizedImage = sizedImage[0:imageHeight-((imageHeight-hog.winHeight) % hog.winStrideY),0:imageWidth-((imageWidth-hog.winWidth) % hog.winStrideX)]
                imageHeight=sizedImage.shape[0]
                imageWidth=sizedImage.shape[1]


            for winY in xrange((imageHeight-hog.winHeight)/hog.winStrideY+1):
                for winX in xrange((imageWidth-hog.winWidth)/hog.winStrideX+1):
                    #print "WinY: " + str(winY) + ", winX: " + str(winX)
                    startY= winY*hog.winStrideY
                    endY= winY*hog.winStrideY +  hog.winHeight
                    startX = winX*hog.winStrideX
                    endX= winX*hog.winStrideX +  hog.winWidth
                    #print (startY,endY,startX,endX)

                    objectCounter=0
                    flag=True
                    obj = imageDetails[1:]
                    cpyImage = sizedImage.copy()
                    cv2.rectangle(cpyImage,(startX,startY),(endX,endY),50)
                    cv2.imshow("Ardrone Front Camera",cpyImage)

                    while flag:
                        for index, o in enumerate(obj):
                            if o[0] > 0:
                                print str(index)+' - '+str(o[0])+", ",
                        key=cv2.waitKey(0)
                        #if key
                        if key>=ord('0') and key<ord(str(len(obj))):
                            i= int(chr(key%256))
                            print i
                            flag=False
                            z=obj[i][0]
                            if z>0:

                                #des=np.append([1],hog.computeBlocks(image[startY:endY,startX:endX]))
                                #np.savetxt(features,des[None], delimiter=',')
                                cv2.imwrite(objectfpath+str(fileCounter)+'-'+str(sc)+'-'+str((winY,winX))+'-'+str(objectCounter)+'-'+str(z)+".png",sizedImage[startY:endY,startX:endX])
                                print str(fileCounter)+'-'+str(sc)+'-'+str((winY,winX))+'-'+str(objectCounter)+'-'+str(z)

                            else:
                                print "Z has value of :" + str(z)+ "do you really want to save feature?"
                                if cv2.waitKey(0)==32:
                                    continue
                                else:
                                    flag=True
                                    #des=np.append([1],hog.computeBlocks(image[startY:endY,startX:endX]))
                                    #np.savetxt(features,des[None], delimiter=',')
                                    cv2.imwrite(objectfpath+str(fileCounter)+'-'+str(sc)+'-'+str((winY,winX))+'-'+str(objectCounter)+'-'+str(abs(z))+".png",sizedImage[startY:endY,startX:endX])
                                    print str(fileCounter)+'-'+str(sc)+'-'+str((winY,winX))+'-'+str(objectCounter)+'-'+str(abs(z))


                        elif key==32:
                            flag=False
                            if random.random()>0.7:
                                #des=np.append([0],hog.computeBlocks(sizedImage[startY:endY,startX:endX]))
                                #np.savetxt(features,des[None], delimiter=',')
                                cv2.imwrite(envfpath+str(fileCounter)+'-'+str(sc)+'-'+str((winY,winX))+'-'+str(objectCounter)+".png",sizedImage[startY:endY,startX:endX])
                                print "Environment feature saved"
                            else:
                                print "Not Saved"

                        else:
                            print "Unkown Input - Please Try Again"

                    #cv2.imshow("Ardrone Front Camera", image[startY:endY,startX:endX])
                    #cv2.waitKey(10)

                    objectCounter+=1

        else:
            #raise CustomException('image Shape - Window Parameter mismatch: Window Parameters adhere to shape values: following conditional statements are in chcked order:')
            print (imageWidth>=hog.winWidth)
            print ((imageWidth-hog.winWidth) % hog.winStrideX)
            print (imageHeight>=hog.winHeight)
            print ((imageHeight-hog.winHeight)% hog.winStrideY)
            #return None
    fileCounter += 1

features.close()
objectc.close()'''

files=glob.glob("/home/constantin/Desktop/env/raw/*.png")
objectCounter=0

for fname in files:

    image=cv2.imread(fname)
    scaleCount=2
    imageHeight=image.shape[0]
    imageWidth=image.shape[1]

	#print (imageWidth,imageHeight
	#for obj in imageDetails[1:]:
	#print obj[1:]
    for sc in xrange(scaleCount):
        sizedImage = cv2.resize(image,(int(imageWidth*pow(hog.scaleFactor,sc)),int(imageHeight*pow(hog.scaleFactor,sc))))
        imageHeight=sizedImage.shape[0]
        imageWidth=sizedImage.shape[1]
        #print (imageWidth,imageHeight)
        #Check that the Window Parameters match the image Dimensions else raise an error quit
        if (imageWidth>=hog.winWidth)  and (imageHeight>=hog.winHeight):

            if not (((imageWidth-hog.winWidth) % hog.winStrideX==0) and ((imageHeight-hog.winHeight) % hog.winStrideY==0)):

                sizedImage = sizedImage[0:imageHeight-((imageHeight-hog.winHeight) % hog.winStrideY),0:imageWidth-((imageWidth-hog.winWidth) % hog.winStrideX)]
                imageHeight=sizedImage.shape[0]
                imageWidth=sizedImage.shape[1]


            for winY in xrange((imageHeight-hog.winHeight)/hog.winStrideY+1):
                for winX in xrange((imageWidth-hog.winWidth)/hog.winStrideX+1):
                    #print "WinY: " + str(winY) + ", winX: " + str(winX)
                    startY= winY*hog.winStrideY
                    endY= winY*hog.winStrideY +  hog.winHeight
                    startX = winX*hog.winStrideX
                    endX= winX*hog.winStrideX +  hog.winWidth
                    #print (startY,endY,startX,endX)

                    cv2.imwrite("/home/constantin/Desktop/env/feature/a-"+str(objectCounter)+".png",sizedImage[startY:endY,startX:endX])

                    objectCounter+=1

        else:
            #raise CustomException('image Shape - Window Parameter mismatch: Window Parameters adhere to shape values: following conditional statements are in chcked order:')
            print (imageWidth>=hog.winWidth)
            print ((imageWidth-hog.winWidth) % hog.winStrideX)
            print (imageHeight>=hog.winHeight)
            print ((imageHeight-hog.winHeight)% hog.winStrideY)
            #return None
    fileCounter += 1

features.close()
objectc.close()