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
import matplotlib.pyplot as plt

import os
# Receiving the video feed
from sensor_msgs.msg import Image

hog=HOG((64,64),(37,32),(32,32),(16,16),(16,16),16,0.5,1)

bf="/home/constantin/ros/ardrone_autonomy/bin/output/features/bowl/features.txt"
df="/home/constantin/ros/ardrone_autonomy/bin/output/features/object/features.txt"

sdf="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/object/features.txt"
sbf="/home/constantin/ros/ardrone_autonomy/bin/output/features/bowl/sample/features.txt"

drillfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/object/*.png"
envfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/environment/*.png"
bowlfpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/bowl/*.png"

sbowlpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/bowl/*.png"
sdrillpath="/home/constantin/ros/ardrone_autonomy/bin/output/features/object/*.png"

envfiles=glob.glob(envfpath)

TESTRATIO=0.2
DRILL=True
BOWL=False

KNN=True
RT=False
SVM=False

SAMPLE=True

BINS=10
histData=np.array([])
envC=0
drillC=0

ENVRATIO=1
OBJRATIO=1

responses=list()
drillData=list()

drillRData=list()
drillZ=list()

if DRILL:
    ot="Drill"
else:
    ot="Bowl"

if KNN:
   dt="KNN"
elif RT:
    dt="RT"
elif SVM:
    dt="SVM"

#content = drillfeatures.read().splitlines()
#content=drillfeatures.readlines()
if DRILL:
    with open(df,"r+") as drillfeatures:
         for line in drillfeatures:#content:
            #print line
            l=eval(line)
            #drill or enviorment identifier
            res=l[0]
            #print res
            #distance
            Z=l[1]

            #print Z
            #distance
            data=l[2:]
            a=len(data)
            #print a


            if res==1 and random.random()<=OBJRATIO:

                responses.append(res)
                drillData.append(data)

                drillRData.append(data)
                drillZ.append(Z)
                drillC+=1
            elif random.random()<=ENVRATIO:

                responses.append(res)
                drillData.append(data)

                envC+=1



    drillfeatures.close()


elif BOWL:
    with open(bf,"r+") as bowlfeatures:
        for line in bowlfeatures:#content:
            #print line
            l=eval(line)
            #drill or enviorment identifier
            res=l[0]
            #print res
            #distance
            Z=l[1]

            #print Z
            #distance
            data=l[2:]
            a=len(data)
            #print a


            if res==1 and random.random()<=OBJRATIO:

                responses.append(res)
                drillData.append(data)

                drillRData.append(data)
                drillZ.append(Z)
                drillC+=1
            elif random.random()<=ENVRATIO:

                responses.append(res)
                drillData.append(data)

                envC+=1

    bowlfeatures.close()

if KNN:

    drillSamples=list()
    responseSamples=list()

    drillRSamples=list()
    drillZSamples=list()

    ENVSIZE=envC
    DRILLSIZE=drillC
    ENVSAMPLESIZE=int(envC*(TESTRATIO))
    DRILLSAMPLESIZE=int(drillC*(TESTRATIO))
    envC=0
    drillC=0

    if SAMPLE:
        print "SAMPLE DATA BEING USED"
        with open(sdf,"r+") as sdrillfeatures:
             for line in sdrillfeatures:#content:
                #print line
                l=eval(line)
                #drill or enviorment identifier
                res=l[0]
                #print res
                #distance
                Z=l[1]

                #print Z
                #distance
                data=l[2:]
                a=len(data)
                #print a

                #store sample and remove from original dataset
                drillSamples.append(data)
                responseSamples.append(res)
                drillRSamples.append(data)
                drillZSamples.append(Z)


        sdrillfeatures.close()
        DRILLSAMPLESIZE=len(responseSamples)
        ENVSAMPLESIZE=1

    else:

        #select sample of files to be test data (removed from train data)
        while envC<ENVSAMPLESIZE and drillC<DRILLSAMPLESIZE:

            #take random index
            index=random.randrange(0,len(responses))

            #if descriptor is that of a drill populate regressor sample too.
            if responses[index]==1 and drillC<DRILLSAMPLESIZE:

                #get index of random regression values
                i=random.randrange(0,len(drillRData))
                drillC+=1

                #store sample and remove from original dataset
                drillRSamples.append(drillRData.pop(i))

                drillZSamples.append(drillZ.pop(i))

                #store sample and remove from original dataset
                drillSamples.append(drillData.pop(index))
                responseSamples.append(responses.pop(index))

            elif envC<ENVSAMPLESIZE:

                envC+=1
                #store sample and remove from original dataset
                drillSamples.append(drillData.pop(index))
                responseSamples.append(responses.pop(index))


    drillData=np.vstack(drillData)
    responses=np.vstack(responses)
    drillZ=np.vstack(drillZ)
    drillRData=np.vstack(drillRData)


    drillSamples=np.vstack(drillSamples)
    responseSamples=np.vstack(responseSamples)

    drillRSamples=np.vstack(drillRSamples)



    drillZSamples=np.vstack(drillZSamples)


    print "Total Data: " + str(ENVSIZE+DRILLSIZE)
    print "Environment Data : " + str(ENVSIZE)
    print "Object Data : " + str(DRILLSIZE) + "\n"

    TRUETOTALSIZE=responses.shape[0]
    TRUEOBJSIZE=np.count_nonzero(responses)
    TRUEENVSIZE=TRUETOTALSIZE-TRUEOBJSIZE

    print "Classification Data: " + str(TRUETOTALSIZE)
    print "Environment Data: " + str(TRUEENVSIZE)
    print "Object Data " + str(TRUEOBJSIZE)+ "\n"

    TRUEREGSIZE=drillZ.shape[0]
    print "Regression Data: " + str(TRUEREGSIZE)+ "\n"

    #print drillZ.shape

    TRUETOTALSAMPLESIZE=responseSamples.shape[0]
    TRUEOBJSAMPLESIZE=np.count_nonzero(responseSamples)

    if SAMPLE:
        TRUEENVSAMPLESIZE=1
    else:
        TRUEENVSAMPLESIZE=TRUETOTALSAMPLESIZE-TRUEOBJSAMPLESIZE

    print "Classification Test Sample: " + str(TRUETOTALSAMPLESIZE)
    print "Environment Data: " + str(TRUEENVSAMPLESIZE)
    print "Object Data " + str(TRUEOBJSAMPLESIZE)+ "\n"

    TRUEREGSAMPLESIZE=drillZSamples.shape[0]

    print "Regression Test Sample: " + str(TRUEREGSAMPLESIZE)+ "\n"



    drillData=np.array(drillData,dtype=np.float32)
    responses=np.array(responses,dtype=np.float32)
    drillZ=np.array(drillZ,dtype=np.float32)
    drillRData=np.array(drillRData,dtype=np.float32)

    drillSamples=np.array(drillSamples,dtype=np.float32)
    responseSamples=np.array(responseSamples,dtype=np.float32)
    drillRSamples=np.array(drillRSamples,dtype=np.float32)
    drillZSamples=np.array(drillZSamples,dtype=np.float32)



    knn = cv2.KNearest()
    knn.train(drillData,responses)

    knnR= cv2.KNearest()
    knnR.train(drillRData,drillZ,isRegression=True)


    envP, envFP, objP, objFP= 0,0,0,0


    ret, results, neighbours, dist = knn.find_nearest(drillSamples, 3)

    #for ind in xrange(len(results)):
    #   print (float(results[i]),float(responseSamples[i]))

    histMax=np.amax(results)
    histRange=histMax/BINS
    histC=0

    for i in xrange(len(results)):

        #if result should be enviorment
        if int(responseSamples[i])==0:

            #increment enviornment Positive and False Positive
            if int(results[i])==0:
                envP+=1
            else:
                objFP+=1


        #if result should be object
        else:
            #increment object Positive and False Positive
            if int(results[i])==1:
                objP+=1
                histData=np.append(histData,drillZSamples[histC])
                histC+=1
            else:
                envFP+=1



    print "Using KNN with a "+ str(TESTRATIO*100) +"% sample size for testing"
    print  "Object Detection Accuracy: "+str(objP)+"/"+str(TRUEOBJSAMPLESIZE)+"(" +  str((float(objP)/TRUEOBJSAMPLESIZE)*100) + "%)"

    print "Enviornment Detection Accuracy: "+str(envP)+"/"+str(TRUEENVSAMPLESIZE)+"(" + str((float(envP)/TRUEENVSAMPLESIZE)*100) + "%)"

    baseHist,baseBins=np.histogram(drillZSamples,BINS)
    trueHist,trueBins=np.histogram(histData,baseBins)
    #print baseHist
    #print baseBins
    #print trueHist
    #print trueBins
    binDiff=np.empty(BINS)
    binDiff.fill(baseBins[1]-baseBins[0])
    binDiffRange=binDiff*np.arange(BINS)
    plt.xticks(binDiffRange)
    #print baseBins[:-1]

    ratioHist=np.array([],dtype=np.float32)
    for i in xrange(BINS):
        if baseHist[i]==0:
            ratioHist=np.append(ratioHist,0)
        else:
            ratioHist=np.append(ratioHist,float(trueHist[i])/baseHist[i])
    #print ratioHist
    plt.bar(baseBins[:-1],ratioHist,binDiff)

    if SAMPLE:
        plt.savefig("/home/constantin/Desktop/project/figures/Detection-"+ot+"-"+dt+"-"+str(SAMPLE)+"NO RESIZE2"+".png")

    else:
        plt.savefig("/home/constantin/Desktop/project/figures/Detection-"+ot+"-"+dt+"-"+str(TESTRATIO)+".png")
        print "Save Second Image"

    plt.draw()  # Draws, but does not block
    #raw_input()  # This shows the first figure "separately" (by waiting for "enter").
    plt.figure()

    #print trueHist/baseHist
    #print np.diff(baseBins)

    #Regressor
    #print drillSamples.shape
    #print type(drillRSamples)
    #print type(drillSamples[0][0])
    #print drillRSamples.shape
    retR,resultsR, neighboursR, distR = knnR.find_nearest(drillRSamples,6)

    #print resultsR.shape

    #calculate the average distance between the sample and calculated distance
    plt.plot(resultsR-drillZSamples,drillZSamples,linestyle='',marker='o')

    if SAMPLE:
        plt.savefig("/home/constantin/Desktop/project/figures/Regression-"+ot+"-"+dt+"-"+str(SAMPLE)+"NO RESIZE2"+".png")


    else:
        plt.savefig("/home/constantin/Desktop/project/figures/Regression-"+ot+"-"+dt+"-"+str(TESTRATIO)+".png")
        print "Save Second Image"

    plt.title('Relation Between Feature Distance and Regressor Prediction')
    plt.ylabel('Actual Distance from Feature')
    plt.xlabel('Difference in Distance From Feature, Actual Distance - Predicted Distance ')
    plt.draw()  # Draws, but does not block

    plt.show()

elif RT:

    drillSamples=list()
    responseSamples=list()

    drillRSamples=list()
    drillZSamples=list()

    ENVSIZE=envC
    DRILLSIZE=drillC
    ENVSAMPLESIZE=int(envC*(TESTRATIO))
    DRILLSAMPLESIZE=int(drillC*(TESTRATIO))
    envC=0
    drillC=0


    if SAMPLE:
        print "SAMPLE DATA BEING USED"
        with open(sdf,"r+") as sdrillfeatures:
             for line in sdrillfeatures:#content:
                #print line
                l=eval(line)
                #drill or enviorment identifier
                res=l[0]
                #print res
                #distance
                Z=l[1]

                #print Z
                #distance
                data=l[2:]
                a=len(data)
                #print a

                #store sample and remove from original dataset
                drillSamples.append(data)
                responseSamples.append(res)
                drillRSamples.append(data)
                drillZSamples.append(Z)


        sdrillfeatures.close()
        DRILLSAMPLESIZE=len(responseSamples)
        ENVSAMPLESIZE=1

    else:

        #select sample of files to be test data (removed from train data)
        while envC<ENVSAMPLESIZE and drillC<DRILLSAMPLESIZE:

            #take random index
            index=random.randrange(0,len(responses))

            #if descriptor is that of a drill populate regressor sample too.
            if responses[index]==1 and drillC<DRILLSAMPLESIZE:

                #get index of random regression values
                i=random.randrange(0,len(drillRData))
                drillC+=1

                #store sample and remove from original dataset
                drillRSamples.append(drillRData.pop(i))

                drillZSamples.append(drillZ.pop(i))

                #store sample and remove from original dataset
                drillSamples.append(drillData.pop(index))
                responseSamples.append(responses.pop(index))

            elif envC<ENVSAMPLESIZE:

                envC+=1
                #store sample and remove from original dataset
                drillSamples.append(drillData.pop(index))
                responseSamples.append(responses.pop(index))


    drillData=np.vstack(drillData)
    responses=np.vstack(responses)
    drillZ=np.vstack(drillZ)
    drillRData=np.vstack(drillRData)


    drillSamples=np.vstack(drillSamples)
    responseSamples=np.vstack(responseSamples)

    drillRSamples=np.vstack(drillRSamples)
    drillZSamples=np.vstack(drillZSamples)


    print "Total Data: " + str(ENVSIZE+DRILLSIZE)
    print "Environment Data : " + str(ENVSIZE)
    print "Object Data : " + str(DRILLSIZE) + "\n"

    TRUETOTALSIZE=responses.shape[0]
    TRUEOBJSIZE=np.count_nonzero(responses)
    TRUEENVSIZE=TRUETOTALSIZE-TRUEOBJSIZE

    print "Classification Data: " + str(TRUETOTALSIZE)
    print "Environment Data: " + str(TRUEENVSIZE)
    print "Object Data " + str(TRUEOBJSIZE)+ "\n"

    TRUEREGSIZE=drillZ.shape[0]
    print "Regression Data: " + str(TRUEREGSIZE)+ "\n"

    #print drillZ.shape

    TRUETOTALSAMPLESIZE=responseSamples.shape[0]
    TRUEOBJSAMPLESIZE=np.count_nonzero(responseSamples)
    if SAMPLE:
        TRUEENVSAMPLESIZE=1
    else:
        TRUEENVSAMPLESIZE=TRUETOTALSAMPLESIZE-TRUEOBJSAMPLESIZE


    print "Classification Test Sample: " + str(TRUETOTALSAMPLESIZE)
    print "Environment Data: " + str(TRUEENVSAMPLESIZE)
    print "Object Data " + str(TRUEOBJSAMPLESIZE)+ "\n"

    TRUEREGSAMPLESIZE=drillZSamples.shape[0]

    print "Regression Test Sample: " + str(TRUEREGSAMPLESIZE)+ "\n"



    drillData=np.array(drillData,dtype=np.float32)
    responses=np.array(responses,dtype=np.float32)
    drillZ=np.array(drillZ,dtype=np.float32)
    drillRData=np.array(drillRData,dtype=np.float32)

    drillSamples=np.array(drillSamples,dtype=np.float32)
    responseSamples=np.array(responseSamples,dtype=np.float32)
    drillRSamples=np.array(drillRSamples,dtype=np.float32)
    drillZSamples=np.array(drillZSamples,dtype=np.float32)

    envP, envFP, objP, objFP= 0,0,0,0

    rt_params = dict(max_depth=10, min_sample_count=5, use_surrogates=False, max_categories=15, calc_var_importance=False, nactive_vars=0, max_num_of_trees_in_the_forest=30, term_crit=(cv2.TERM_CRITERIA_MAX_ITER,30,1))

    rt = cv2.RTrees()
    rt.train(drillData, cv2.CV_ROW_SAMPLE, responses, params=rt_params)


    results=np.array([],dtype=np.float32)
    for i in xrange(drillSamples.shape[0]):
        results=np.append(results, rt.predict(drillSamples[i]))



    histMax=np.amax(results)
    histRange=histMax/BINS
    histC=0

    for i in xrange(len(results)):

        #if result should be enviorment
        if int(responseSamples[i])==0:

            #increment enviornment Positive and False Positive
            if int(results[i])==0:
                envP+=1
            else:
                objFP+=1


        #if result should be object
        else:
            #increment object Positive and False Positive
            if int(results[i])==1:
                objP+=1
                histData=np.append(histData,drillZSamples[histC])
                histC+=1
            else:
                envFP+=1



    print "Using KNN with a "+ str(TESTRATIO*100) +"% sample size for testing"
    print  "Object Detection Accuracy: "+str(objP)+"/"+str(TRUEOBJSAMPLESIZE)+"(" +  str((float(objP)/TRUEOBJSAMPLESIZE)*100) + "%)"

    print "Enviornment Detection Accuracy: "+str(envP)+"/"+str(TRUEENVSAMPLESIZE)+"(" + str((float(envP)/TRUEENVSAMPLESIZE)*100) + "%)"

    baseHist,baseBins=np.histogram(drillZSamples,BINS)
    trueHist,trueBins=np.histogram(histData,baseBins)
    #print baseHist
    #print baseBins
    #print trueHist
    #print trueBins
    binDiff=np.empty(BINS)
    binDiff.fill(baseBins[1]-baseBins[0])
    binDiffRange=binDiff*np.arange(BINS)
    plt.xticks(binDiffRange)
    #print baseBins[:-1]

    ratioHist=np.array([],dtype=np.float32)
    for i in xrange(BINS):
        if baseHist[i]==0:
            ratioHist=np.append(ratioHist,0)
        else:
            ratioHist=np.append(ratioHist,float(trueHist[i])/baseHist[i])
    #print ratioHist
    plt.bar(baseBins[:-1],ratioHist,binDiff)

    if SAMPLE:
        plt.savefig("/home/constantin/Desktop/project/figures/Detection-"+ot+"-"+dt+"-"++str(SAMPLE)+".png")


    else:
        plt.savefig("/home/constantin/Desktop/project/figures/Detection-"+ot+"-"+dt+"-"+str(TESTRATIO)+".png")


    plt.draw()  # Draws, but does not block
    #raw_input()  # This shows the first figure "separately" (by waiting for "enter").

    '''rtR_params = dict(max_depth=10, min_sample_count=5, use_surrogates=False, max_categories=15, calc_var_importance=False, nactive_vars=0, max_num_of_trees_in_the_forest=30, term_crit=(cv2.TERM_CRITERIA_MAX_ITER,30,1),varType=(cv2.CV_VAR_ORDERED),regression_accuracy=0.0001)

    rtR= cv2.RTrees()
    rtR.train(drillRData,cv2.CV_ROW_SAMPLE, drillZ,params=rtR_params)

    retR=rt.predict(drillZSamples[0])'''
    plt.show()


elif SVM:

    drillSamples=list()
    responseSamples=list()

    drillRSamples=list()
    drillZSamples=list()

    ENVSIZE=envC
    DRILLSIZE=drillC
    ENVSAMPLESIZE=int(envC*(TESTRATIO))
    DRILLSAMPLESIZE=int(drillC*(TESTRATIO))
    envC=0
    drillC=0

    if SAMPLE:
        print "SAMPLE DATA BEING USED"
        with open(sdf,"r+") as sdrillfeatures:
             for line in sdrillfeatures:#content:
                #print line
                l=eval(line)
                #drill or enviorment identifier
                res=l[0]
                #print res
                #distance
                Z=l[1]

                #print Z
                #distance
                data=l[2:]
                a=len(data)
                #print a

                #store sample and remove from original dataset
                drillSamples.append(data)
                responseSamples.append(res)
                drillRSamples.append(data)
                drillZSamples.append(Z)


        sdrillfeatures.close()
        DRILLSAMPLESIZE=len(responseSamples)
        ENVSAMPLESIZE=1

    else:

        #select sample of files to be test data (removed from train data)
        while envC<ENVSAMPLESIZE and drillC<DRILLSAMPLESIZE:

            #take random index
            index=random.randrange(0,len(responses))

            #if descriptor is that of a drill populate regressor sample too.
            if responses[index]==1 and drillC<DRILLSAMPLESIZE:

                #get index of random regression values
                i=random.randrange(0,len(drillRData))
                drillC+=1

                #store sample and remove from original dataset
                drillRSamples.append(drillRData.pop(i))

                drillZSamples.append(drillZ.pop(i))

                #store sample and remove from original dataset
                drillSamples.append(drillData.pop(index))
                responseSamples.append(responses.pop(index))

            elif envC<ENVSAMPLESIZE:

                envC+=1
                #store sample and remove from original dataset
                drillSamples.append(drillData.pop(index))
                responseSamples.append(responses.pop(index))


    drillData=np.vstack(drillData)
    responses=np.vstack(responses)
    drillZ=np.vstack(drillZ)
    drillRData=np.vstack(drillRData)


    drillSamples=np.vstack(drillSamples)
    responseSamples=np.vstack(responseSamples)

    drillRSamples=np.vstack(drillRSamples)
    drillZSamples=np.vstack(drillZSamples)


    print "Total Data: " + str(ENVSIZE+DRILLSIZE)
    print "Environment Data : " + str(ENVSIZE)
    print "Object Data : " + str(DRILLSIZE) + "\n"

    TRUETOTALSIZE=responses.shape[0]
    TRUEOBJSIZE=np.count_nonzero(responses)
    TRUEENVSIZE=TRUETOTALSIZE-TRUEOBJSIZE

    print "Classification Data: " + str(TRUETOTALSIZE)
    print "Environment Data: " + str(TRUEENVSIZE)
    print "Object Data " + str(TRUEOBJSIZE)+ "\n"

    TRUEREGSIZE=drillZ.shape[0]
    print "Regression Data: " + str(TRUEREGSIZE)+ "\n"

    #print drillZ.shape

    TRUETOTALSAMPLESIZE=responseSamples.shape[0]
    TRUEOBJSAMPLESIZE=np.count_nonzero(responseSamples)
    if SAMPLE:
        TRUEENVSAMPLESIZE=1
    else:
        TRUEENVSAMPLESIZE=TRUETOTALSAMPLESIZE-TRUEOBJSAMPLESIZE

    print "Classification Test Sample: " + str(TRUETOTALSAMPLESIZE)
    print "Environment Data: " + str(TRUEENVSAMPLESIZE)
    print "Object Data " + str(TRUEOBJSAMPLESIZE)+ "\n"

    TRUEREGSAMPLESIZE=drillZSamples.shape[0]

    print "Regression Test Sample: " + str(TRUEREGSAMPLESIZE)+ "\n"



    drillData=np.array(drillData,dtype=np.float32)
    responses=np.array(responses,dtype=np.float32)
    drillZ=np.array(drillZ,dtype=np.float32)
    drillRData=np.array(drillRData,dtype=np.float32)

    drillSamples=np.array(drillSamples,dtype=np.float32)
    responseSamples=np.array(responseSamples,dtype=np.float32)
    drillRSamples=np.array(drillRSamples,dtype=np.float32)
    drillZSamples=np.array(drillZSamples,dtype=np.float32)



    svm_params = dict( kernel_type = cv2.SVM_LINEAR,svm_type = cv2.SVM_C_SVC)

    svm = cv2.SVM()
    svm.train(drillData,responses,params=svm_params)

    svmR_params=dict( kernel_type = cv2.SVM_LINEAR,svm_type = cv2.SVM_EPS_SVR,p=10)

    svmR= cv2.SVM()
    svmR.train(drillRData,drillZ,params=svmR_params)


    envP, envFP, objP, objFP= 0,0,0,0


    results = svm.predict_all(drillSamples)

    #for ind in xrange(len(results)):
    #   print (float(results[i]),float(responseSamples[i]))



    histMax=np.amax(results)
    histRange=histMax/BINS
    histC=0

    for i in xrange(len(results)):

        #if result should be enviorment
        if int(responseSamples[i])==0:

            #increment enviornment Positive and False Positive
            if int(results[i])==0:
                envP+=1
            else:
                objFP+=1


        #if result should be object
        else:
            #increment object Positive and False Positive
            if int(results[i])==1:
                objP+=1
                histData=np.append(histData,drillZSamples[histC])
                histC+=1
            else:
                envFP+=1



    print "Using KNN with a "+ str(TESTRATIO*100) +"% sample size for testing"
    print  "Object Detection Accuracy: "+str(objP)+"/"+str(TRUEOBJSAMPLESIZE)+"(" +  str((float(objP)/TRUEOBJSAMPLESIZE)*100) + "%)"

    print "Enviornment Detection Accuracy: "+str(envP)+"/"+str(TRUEENVSAMPLESIZE)+"(" + str((float(envP)/TRUEENVSAMPLESIZE)*100) + "%)"

    baseHist,baseBins=np.histogram(drillZSamples,BINS)
    trueHist,trueBins=np.histogram(histData,baseBins)
    #print baseHist
    #print baseBins
    #print trueHist
    #print trueBins
    binDiff=np.empty(BINS)
    binDiff.fill(baseBins[1]-baseBins[0])
    binDiffRange=binDiff*np.arange(BINS)
    plt.xticks(binDiffRange)
    #print baseBins[:-1]

    ratioHist=np.array([],dtype=np.float32)
    for i in xrange(BINS):
        if baseHist[i]==0:
            ratioHist=np.append(ratioHist,0)
        else:
            ratioHist=np.append(ratioHist,float(trueHist[i])/baseHist[i])
    #print ratioHist
    plt.bar(baseBins[:-1],ratioHist,binDiff)



    if SAMPLE:
        plt.savefig("/home/constantin/Desktop/project/figures/Detection-"+ot+"-"+dt+"-"+str(SAMPLE)+".png")
        plt.close()

    else:
        plt.savefig("/home/constantin/Desktop/project/figures/Detection-"+ot+"-"+dt+"-"+str(TESTRATIO)+".png")
        print "Save Image"
        plt.close()
    #plt.draw()  # Draws, but does not block
    #raw_input()  # This shows the first figure "separately" (by waiting for "enter").


    fig=plt.figure()


    #print trueHist/baseHist
    #print np.diff(baseBins)

    #Regressor
    #print drillSamples.shape
    resultsR = svmR.predict_all(drillRSamples)

    #print resultsR.shape

    #calculate the average distance between the sample and calculated distance
    plt.plot(resultsR-drillZSamples,drillZSamples,linestyle='',marker='o')

    plt.title('Relation Between Feature Distance and Regressor Prediction')
    plt.ylabel('Actual Distance from Feature')
    plt.xlabel('Difference in Distance From Feature, Actual Distance - Predicted Distance ')


    if SAMPLE:
        plt.savefig("/home/constantin/Desktop/project/figures/Regression-"+ot+"-"+dt+"-"+str(SAMPLE)+".png")


    else:
        plt.savefig("/home/constantin/Desktop/project/figures/Regression-"+ot+"-"+dt+"-"+str(TESTRATIO)+".png")


    #plt.draw()  # Draws, but does not block


    #plt.show()