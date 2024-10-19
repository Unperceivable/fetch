#!/usr/bin/env python


import cv2
import numpy as np
import matplotlib.pyplot as plt
import random

#import HOG class
from hogv0 import HOG

from random import uniform
import glob
import random
import os
# Receiving the video feed
from sensor_msgs.msg import Image

hog=HOG((64,64),(37,32),(32,32),(16,16),(16,16),16,0.5,1)

bf="/home/constantin/ros/ardrone_autonomy/bin/output/features/bowl/features.txt"
df="/home/constantin/ros/ardrone_autonomy/bin/output/features/object/features.txt"

drillfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/object/*.png"
envfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/environment/*.png"
bowlfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/bowl/*.png"


sdf="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/object/features.txt"
sbf="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/bowl/features.txt"

sbowlfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/bowl/*.png"
sdrillfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/object/*.png"
senvfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/environment/*.png"

envfiles=glob.glob(envfpath)

TESTRATIO=0.2
DRILL=True
BOWL=False

SAMPLE=True


if DRILL:
    drillfeatures = open(df,"w+")
    drillfiles=glob.glob(drillfpath)


    #for train data in test files
    for fname in drillfiles:

        image=cv2.imread(fname)

        name=fname.split('-')

        #name[1]= scale counter, name[4]=distance to object z
        #scale counter = 0 (no resize) 1=rescaled by hog scale factor (default=0.5) = double distance
        #print "name"+ str(name[4].rsplit('.',1)[0])


        #calculate rescaled distance from drone to object in feature for distance recognition metric
        #divide z by 1 or 0.5
        dis=float(name[4].rsplit('.',1)[0])/(pow(hog.scaleFactor,int(name[1])) )
        #print "dist"+str(dis)

        des=np.append([1,dis],hog.computeBlocks(image))
        np.savetxt(drillfeatures,des[None], delimiter=',')

    for fname in envfiles:
        image=cv2.imread(fname)
        des=np.append([0,0],hog.computeBlocks(image))
        np.savetxt(drillfeatures,des[None], delimiter=',')

    drillfeatures.close()


if BOWL:
    bowlfeatures = open(bf,"w+")
    bowlfiles=glob.glob(bowlfpath)

    for fname in bowlfiles:

        image=cv2.imread(fname)

        name=fname.split('-')
        #name[1]= scale counter, name[4]=distance to object z
        #scale counter = 0 (no resize) 1=rescaled by hog scale factor (default=0.5) = double distance
        #calculate rescaled distance from drone to object in feature for distance recognition metric
        #divide z by 1 or 0.5
        dis=float(name[4].rsplit('.',1)[0])/(pow(hog.scaleFactor,int(name[1])) )
        des=np.append([1,dis],hog.computeBlocks(image))
        np.savetxt(bowlfeatures,des[None], delimiter=',')

    for fname in envfiles:
        des=np.append([0,0],hog.computeBlocks(image))
        np.savetxt(bowlfeatures,des[None], delimiter=',')

    bowlfeatures.close()


if SAMPLE:

    sdrillfeatures = open(sdf,"w+")
    sdrillfiles=glob.glob(sdrillfpath)


    #for train data in test files
    for fname in sdrillfiles:

        image=cv2.imread(fname)

        name=fname.split('-')

        #name[1]= scale counter, name[4]=distance to object z
        #scale counter = 0 (no resize) 1=rescaled by hog scale factor (default=0.5) = double distance
        #print "name"+ str(name[4].rsplit('.',1)[0])


        #calculate rescaled distance from drone to object in feature for distance recognition metric
        #divide z by 1 or 0.5
        dis=float(name[4].rsplit('.',1)[0])/(pow(hog.scaleFactor,int(name[1])) )
        #print "dist"+str(dis)

        #if name[1]=="0":
        des=np.append([1,dis],hog.computeBlocks(image))
        np.savetxt(sdrillfeatures,des[None], delimiter=',')

    envfiles=glob.glob(senvfpath)
    for fname in envfiles:
        des=np.append([0,0],hog.computeBlocks(image))
        np.savetxt(sdrillfeatures,des[None], delimiter=',')

    sdrillfeatures.close()


    sbowlfeatures = open(sbf,"w+")
    sbowlfiles=glob.glob(sbowlfpath)

    for fname in sbowlfiles:

        image=cv2.imread(fname)

        name=fname.split('-')
        #name[1]= scale counter, name[4]=distance to object z
        #scale counter = 0 (no resize) 1=rescaled by hog scale factor (default=0.5) = double distance
        #calculate rescaled distance from drone to object in feature for distance recognition metric
        #divide z by 1 or 0.5
        dis=float(name[4].rsplit('.',1)[0])/(pow(hog.scaleFactor,int(name[1])) )
        des=np.append([1,dis],hog.computeBlocks(image))
        np.savetxt(sbowlfeatures,des[None], delimiter=',')

    envfiles=glob.glob(senvfpath)
    for fname in envfiles:
        des=np.append([0,0],hog.computeBlocks(image))
        np.savetxt(sbowlfeatures,des[None], delimiter=',')

    sbowlfeatures.close()

