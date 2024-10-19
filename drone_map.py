#!/usr/bin/env python



import cv2
import cv2.cv as cv
import numpy as np
import random
import math

#import HOG class
from hogv0 import HOG
from geometry_msgs.msg import PoseArray, Pose
import glob


class DroneMap():

    def __init__(self):


        self.hog=HOG((64,64),(37,32),(32,32),(16,16),(16,16),16,0.5,1)

        #self.of="/home/constantin/ros/ardrone_autonomy/bin/output/features/bowl/features.txt"

        #self.sof="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/bowl/features.txt"

        self.of="/home/constantin/ros/ardrone_autonomy/bin/output/features/object/features.txt"
        self.sof="/home/constantin/ros/ardrone_autonomy/bin/output/features/sample/object/features.txt"

        self.detector=0

        self.mapWidth=5
        self.mapLength=7.5

        self.gridResolution=0.25
        self.voxelResolution=0.75

        self.droneOrigin=(3.0,2.5)

        self.REGRESSION=False
        self.DISTANCE=False

        self.scaleFactor=15
        self.decayFactor=0.9
        self.distanceFactor=0.9

        self.probabilityMap=np.zeros((int(self.mapLength/self.gridResolution),int(self.mapWidth/self.gridResolution),3))
        self.probabilityMap.fill(0.01)

        self.visualMap=np.zeros((self.probabilityMap.shape[0]*self.scaleFactor,self.probabilityMap.shape[1]*self.scaleFactor), np.uint8)

    def Train(self):

        envC=0
        objC=0
        sobjC=0
        senvC=0

        responses=list()
        drillData=list()

        drillRData=list()
        drillZ=list()

        drillSamples=list()
        responseSamples=list()

        drillRSamples=list()
        drillZSamples=list()

        print "Loading Training Data"

        with open(self.of,"r+") as objfeatures:
             for line in objfeatures:#content:
                #print line
                l=eval(line)
                #obj or enviorment identifier
                res=l[0]
                #print res
                #distance
                Z=l[1]

                #print Z
                #distance
                data=l[2:]
                a=len(data)
                #print a


                if res==1:

                    responses.append(res)
                    drillData.append(data)

                    drillRData.append(data)
                    drillZ.append(Z)
                    objC+=1
                else:

                    responses.append(res)
                    drillData.append(data)

                    envC+=1


        objfeatures.close()

        print "Loading Sample Data"

        with open(self.sof,"r+") as sdrillfeatures:
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


                if res==1:
                    drillSamples.append(data)
                    responseSamples.append(res)

                    drillRSamples.append(data)
                    drillZSamples.append(Z)
                    sobjC+=1

                else:
                    drillSamples.append(data)
                    responseSamples.append(res)

                    senvC+=1


        sdrillfeatures.close()

        drillData=np.vstack(drillData)
        responses=np.vstack(responses)

        drillZ=np.vstack(drillZ)
        drillRData=np.vstack(drillRData)

        drillSamples=np.vstack(drillSamples)
        responseSamples=np.vstack(responseSamples)

        drillRSamples=np.vstack(drillRSamples)
        drillZSamples=np.vstack(drillZSamples)

        drillData=np.array(drillData,dtype=np.float32)
        responses=np.array(responses,dtype=np.float32)
        drillZ=np.array(drillZ,dtype=np.float32)
        drillRData=np.array(drillRData,dtype=np.float32)

        drillSamples=np.array(drillSamples,dtype=np.float32)
        responseSamples=np.array(responseSamples,dtype=np.float32)
        drillRSamples=np.array(drillRSamples,dtype=np.float32)
        drillZSamples=np.array(drillZSamples,dtype=np.float32)

        print "Object,Enviornment Features Loaded" + str((objC,envC))

        print "Training and Result Generation"

        #KNN
        if self.detector==0:

            self.knn = cv2.KNearest()
            self.knn.train(drillData,responses)

            self.knnR = cv2.KNearest()
            self.knnR.train(drillRData,drillZ,isRegression=True)


            objP,envP,objFP,envFP=0,0,0,0

            ret, results, neighbours, dist = self.knn.find_nearest(drillSamples, 3)



            for i in xrange(len(results)):

                #if result should be object
                if int(responseSamples[i])==1:
                    #increment object Positive and False Positive
                    if int(results[i])==1:
                        objP+=1

                    else:
                        envFP+=1
                else:
                    if int(results[i])==0:
                        envP+=1
                    else:
                        objFP+=1

            print "Object,Enviornment Samples Loaded: " + str((sobjC,senvC))

            print "Object Detection Rate: " + str(objP) + "/" + str(sobjC) + " = " + str(float(objP)/sobjC)

            print "Environment Detection Rate: " +str(envP) + "/" + str(senvC) + " = " + str(float(envP)/senvC)

            gamma=float(objP)+float(envP)
            delta=float(objFP)+float(envFP)

            print "Gamma / Delta: " + str(gamma) + "/" + str(delta) + " = " + str(float(gamma)/delta)


            if delta!=0:
                self.gd=gamma/delta
            else:
                print "DESCRIPTOR IS PERFECT?!"
                self.gd=2

            retR, resultsR, neighboursR, distR = self.knnR.find_nearest(drillRSamples, 3)

            regression=resultsR-drillZSamples

            print type(resultsR)
            print type(drillZSamples)
            print resultsR.shape
            print drillZSamples.shape

            regressionBin0=[]
            regressionBin1=[]
            regressionBin2=[]
            a0=[]
            a1=[]
            a2=[]


            for i in xrange(resultsR.shape[0]):
                #close
                if drillZSamples[i]<1:
                    regressionBin0.append(regression[i])
                    a0.append(resultsR[i])
                #midrange
                elif drillZSamples[i]<2.5:
                    regressionBin1.append(regression[i])
                    a1.append(resultsR[i])
                #far
                else:
                    regressionBin2.append(regression[i])
                    a2.append(resultsR[i])

            regressionBin0=np.array(regressionBin0,dtype=np.float32)
            regressionBin1=np.array(regressionBin1,dtype=np.float32)
            regressionBin2=np.array(regressionBin2,dtype=np.float32)


            self.regressionMean0=np.mean(regressionBin0)
            self.regressionStd0=np.std(regressionBin0)

            self.regressionMean1=np.mean(regressionBin1)
            self.regressionStd1=np.std(regressionBin1)

            self.regressionMean2=np.mean(regressionBin2)
            self.regressionStd2=np.std(regressionBin2)




            print "------------------"
            print self.regressionMean0
            print self.regressionStd0
            print "------------------"
            print self.regressionMean1
            print self.regressionStd1
            print "------------------"
            print self.regressionMean2
            print self.regressionStd2


            a0=np.array(a0,dtype=np.float32)
            a1=np.array(a1,dtype=np.float32)
            a2=np.array(a2,dtype=np.float32)


            a0=np.mean(a0)
            aStd0=np.std(a0)

            a1=np.mean(a1)
            aStd1=np.std(a1)

            a2=np.mean(a2)
            a2Std2=np.std(a2)

            '''print "******************"
            print self.a0
            print self.aStd0
            print "******************"
            print self.a1
            print self.aStd1
            print "******************"
            print self.a2
            print self.aStd2'''



        #RF
        else:

            objP,envFP=0,0

            rt_params = dict(max_depth=10, min_sample_count=5, use_surrogates=False, max_categories=15, calc_var_importance=False, nactive_vars=0, max_num_of_trees_in_the_forest=30, term_crit=(cv2.TERM_CRITERIA_MAX_ITER,30,1))

            self.rt = cv2.RTrees()
            self.rt.train(drillData, cv2.CV_ROW_SAMPLE, responses, params=rt_params)


            results=np.array([],dtype=np.float32)
            for i in xrange(drillSamples.shape[0]):
                results=np.append(results, self.rt.predict(drillSamples[i]))

            for i in xrange(len(results)):
                #if result should be object
                if int(responseSamples[i])==1:
                    #increment object Positive and False Positive
                    if int(results[i])==1:
                        objP+=1

                    else:
                        envFP+=1


            gamma=objP
            delta=sobjC-objP

            self.gd=gamma/delta

    def Update(self,image,cameraInfo,rotationMatrix,dronePose,c):

        windowCenters,hogData=self.computeImage(image)

        '''print "-----------"
        print type(hogData[0])
        print "-----------"'''
        #a= -R^-1 * t
        #t=[dx,dy,dz]'

        windowPoints=[]

        R=rotationMatrix.getI()

        #print rotationMatrix

        #print dronePose
        #starting point x=width, y=height, z=depth
        #a = -R*dronePose
        a = dronePose
        #a[0]=-dy
        #a[1]=-dz
        #a[2]=-dx

        x,y,z=(3-a[0],2.5-a[1],0+a[2])
        #print "direction"
        #print [[dx],[dy],[dz]]

        self.dronePose=(x,y,z)

        #print a


        for imagePoint in windowCenters:
            #d=focal length

            #u' = u-cx
            #v' = v-cy
            u,v = ( (imagePoint[0]-cameraInfo[0,2])/cameraInfo[0,0],(imagePoint[1]-cameraInfo[1,2])/cameraInfo[1,1] )


            #b=R^-1 * [u'/d , v'/d , 1]'

            #unit direction, in x (left right), y(up down) z(foward)
            b=R*c.getI()*np.matrix([[u],[v],[1]])

            #value in map 2d coordinates
            #b[0]=-dy
            #b[1]=-dz
            #b[2]=-dx



            #dx,dy,dz = (-b[2],-b[0],-b[1])
            dx,dy,dz = (b[0],b[1],b[2])
            #dx's sign shouldnt be negative but works better if is.

            #print (dx,dy,dz)


            #calculate the limiting factor in the enviornment
            lSx = x/dx
            lSy = y/dy
            lSz = -z/dz

            #upper enviornment limint s calculation
            uSx = (x-self.mapLength)/dx
            uSy = (y-self.mapWidth)/dy

            #print "----------------------"
            #print (lSx,lSy,lSz,uSx,uSy)

            s = [e for e in [lSx,lSy,lSz,uSx,uSy] if e > 0]

            #calculate the maximum amount of steps until ray reachs the enviornment boundries
            s=min(s)

            #print s


            startPoint = (float(x),float(y),float(z))
            endPoint = (float(startPoint[0] - dx*s),float(startPoint[1] - dy*s),float(startPoint[1] + dz*s))

            windowPoints.append((startPoint,endPoint))

            #print "----------------"
            #print "Pos at Time Step:"
            #print "Coord Difference: "+ str((abs(float(a[0]-dronePose[0])),abs(float(a[1]-dronePose[1])),abs(float(a[2]-dronePose[2]))))
            #print b
            #print (startPoint,endPoint)
            #print "----------------"


            #print startPoint
            ##print endPoint
            #print (dx,dy,dz)
            #print "-------------------------"

        '''print "-------------"
        print windowCenters[0]
        print windowCenters[-1]
        print windowPoints[0]
        print windowPoints[-1]
        print "-------------"'''

        #print windowPoints[-1]

        cellcoords=self.computePoints(windowPoints,hogData)

        #self.Draw(cellcoords)
        self.Display(cellcoords,hogData)

    def Display(self,sortedpointcoords,hogData):


        meanProb=np.mean(self.probabilityMap)

        hogData=np.vstack(hogData)

        hogData=np.array(hogData,dtype=np.float32)

        if self.detector==0:

            ret, results, neighbours, dist = self.knn.find_nearest(hogData, 3)

        if len(results)==len(sortedpointcoords):
            for i in xrange(len(results)):

                if int(results[i])==1:
                    #for each cell affected by positive detection window
                    for cell in sortedpointcoords[i]:
                        if cell[2]<2:
                            self.probabilityMap[cell[0]][cell[1]][cell[2]]*=self.gd

                            #maintain normilization
                            if self.probabilityMap[cell[0]][cell[1]][cell[2]]>1:
                                self.probabilityMap[cell[0]][cell[1]][cell[2]]=1


                        else:
                            self.probabilityMap[cell[0]][cell[1]][2]*=self.gd
                            #maintain normilization
                            if self.probabilityMap[cell[0]][cell[1]][2]>1:
                                self.probabilityMap[cell[0]][cell[1]][2]=1

                            if self.DISTANCE:
                                self.probabilityMap[cell[0]][cell[1]][2]*=self.distanceFactor


                else:
                    #for each cell viewable with no detection decay value
                    for cell in sortedpointcoords[i]:
                        if cell[2]<2:
                            if self.probabilityMap[cell[0]][cell[1]][cell[2]]>meanProb:
                                self.probabilityMap[cell[0]][cell[1]][cell[2]]*=self.decayFactor



                        else:
                            if self.probabilityMap[cell[0]][cell[1]][2]>meanProb:
                                self.probabilityMap[cell[0]][cell[1]][2]*=self.decayFactor

                            if self.DISTANCE:
                                self.probabilityMap[cell[0]][cell[1]][2]*=self.distanceFactor

        else:
            print "lists dont have the same dimensions"
            print "sorted: " + str(len(sortedpointcoords))
            print "results: " + str(len(results))


        #visualMap = self.visualMap.copy()

        '''for mY in xrange(self.probabilityMap.shape[0]):
            for mX in xrange(self.probabilityMap.shape[1]):
                startY = mY * self.scaleFactor
                endY = mY * self.scaleFactor + self.scaleFactor
                startX = mX * self.scaleFactor
                endX = mX * self.scaleFactor + self.scaleFactor
self.probabilityMap

                if len(sortedpointcoords)>0 and sortedpointcoords[0][0]==mY and sortedpointcoords[0][1]==mX:
                    #cv2.rectangle(visualMap,(startX,startY),(endX,endY),int(self.probabilityMap[mY][mX]*255),thickness=cv2.cv.CV_FILLED)
                    #sortedpointcoords.pop(0)
                else:
                    #cv2.rectangle(visualMap,(startX,startY),(endX,endY),int(self.probabilityMap[mY][mX]*255),thickness=cv2.cv.CV_FILLED)'''



        #print self.probabilityMap

        #cv2.imshow("PMAP",visualMap)

        #visualMap=cv2.cvtColor(visualMap,cv2.COLOR_GRAY2RGB)
        #visualMap=cv2.cvtColor(visualMap,cv2.cv.CV_RGB2Luv)


        cv2.imshow("Low-PMAP",cv2.resize(self.probabilityMap[:,:,0]*255,None,fx=15,fy=15,interpolation=cv2.INTER_NEAREST))
        #cv2.imshow("Mid-PMAP",cv2.resize(self.probabilityMap[:,:,1]*255,None,fx=15,fy=15,interpolation=cv2.INTER_NEAREST))
        #cv2.imshow("High-PMAP",cv2.resize(self.probabilityMap[:,:,2]*255,None,fx=15,fy=15,interpolation=cv2.INTER_NEAREST))

        cv2.imshow("PMAP",cv2.resize(self.probabilityMap,None,fx=15,fy=15,interpolation=cv2.INTER_NEAREST))


        '''colorMapping=(1,0.5,0.5)

        visualMap[:,:,0]*=colorMapping[0]
        visualMap[:,:,1]*=colorMapping[1]
        visualMap[:,:,2]*=colorMapping[2]

        cv2.imshow("CMAP",cv2.resize(visualMap, (0,0), fx=5, fy=5))'''
        #cv2.imshow("CMAP",visualMap)
        cv2.waitKey(10)

    def Draw(self,points):


        '''(x0,y0) = startPoint
        (x1,y1) = endPoint
        "Bresenham's line algorithm"

        points=[]

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -self.gridResolution if x0 > x1 else self.gridResolution
        sy = -self.gridResolution if y0 > y1 else self.gridResolution

        if dx > dy:
            err = dx /(2*self.gridResolution)
            while x != x1-(self.gridResolution*0.5) or x != x1+ (self.gridResolution*0.5):
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy /(2*self.gridResolution)
            while y != y1-(self.gridResolution*0.5) or y != y1+(self.gridResolution*0.5):
                points.append((x,y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy'''




        #print sortedpointcoords

        #print self.probabilityMap.shape

        coordlist=[]
        #print "-------------------"
        #print "points"
        #print points
        #print "end of points"
        for l in points:
            #print l
            for e in l:
                coordlist.append(e)

        #print coordset

        pointcoords=list(set(coordlist))
        #print pointcoords
        sortedpointcoords=pointcoords
        #sortedpointcoords=sorted(pointcoords,key=lambda x: (x[0], x[1]))
        #print sortedpointcoords
        #print "---------------------"
        #self.visualMap=self.probabilityMap*255
        visualMap=np.zeros((self.probabilityMap.shape[0],self.probabilityMap.shape[1],self.probabilityMap.shape[2]))


        for mY in xrange(self.probabilityMap.shape[0]):
            for mX in xrange(self.probabilityMap.shape[1]):

                if len(sortedpointcoords)>0 and (mY,mX,0) in sortedpointcoords:
                    visualMap[mY][mX][0]=255

                if len(sortedpointcoords)>0 and (mY,mX,1) in sortedpointcoords:
                    visualMap[mY][mX][1]=255


                if len(sortedpointcoords)>0 and (mY,mX,2) in sortedpointcoords:
                    visualMap[mY][mX][2]=255

                elif len(sortedpointcoords)>0 and (mY,mX,3) in sortedpointcoords:
                    visualMap[mY][mX][2]=255
                    #print "Unusual values"

                elif len(sortedpointcoords)>0 and (mY,mX,4) in sortedpointcoords:
                    visualMap[mY][mX][2]=255
                    print "SUPER Unusual values"


                #    visualMap[mY][mX]=255
                #else:
                #    #visualMap[mY][mX]=125



        #convert wold coordinate to map coordinate
        #MapLength = WorldX, MapWidth=WorldY


        #startPoint=(self.visualMap.shape[0]*startPoint[1]/self.mapLength,self.visualMap.shape[1]*startPoint[0]/self.mapWidth)
        #endPoint=(self.visualMap.shape[0]*endPoint[1]/self.mapLength,self.visualMap.shape[1]*endPoint[0]/self.mapWidth)

        #calculate grids line passes through using bresenham's line algorithm

        #visualMap=cv2.resize(visualMap, (visualMap.shape[1]*self.scaleFactor,visualMap.shape[0]*self.scaleFactor),interpolation=cv2.INTER_AREA)
        '''cv2.line(visualMap,startPoint,endPoint,130)
        rows,cols=visualMap.shape
        M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
        copyMap = cv2.warpAffine(visualMap,M,(cols,rows))'''
        #print visualMap

        #cv2.imshow("Low-FOV",cv2.resize(visualMap[:,:,0],None,fx=15,fy=15,interpolation=cv2.INTER_NEAREST))
        #cv2.imshow("Mid-FOV",cv2.resize(visualMap[:,:,1],None,fx=15,fy=15,interpolation=cv2.INTER_NEAREST))
        #cv2.imshow("High-FOV",cv2.resize(visualMap[:,:,2],None,fx=15,fy=15,interpolation=cv2.INTER_NEAREST))
        cv2.imshow("FOV",cv2.resize(visualMap,None,fx=15,fy=15,interpolation=cv2.INTER_NEAREST))

        #print (startPoint,endPoint)

        cv2.waitKey(10)


    def computePoints(self,windowPoints,hogData):
        #print windowPoints

        if self.REGRESSION:

            retR, resultsR, neighboursR, distR = self.knnR.find_nearest(hogData, 3)

        cellcoords=[]

        for i in xrange(len(windowPoints)):

            regFlag=False
            disFlag=False

            regMean=0
            regStd=0

            startPoint=windowPoints[i][0]
            endPoint=windowPoints[i][1]

            points=[]

            (x0,y0,z0) = startPoint
            (x1,y1,z1) = endPoint

            dx = abs(x0-x1)
            dy = abs(y0-y1)
            dz = abs(z0-z1)




            if self.REGRESSION:
                reg=resultsR[i]

                #close
                if reg-self.regressionMean0<1:

                    #regression value minus mean gives center of search ray
                    reg=reg-self.regressionMean0
                    #1 standard deviation minus center gives start point
                    x0=reg-self.regressionMean0

                    #two standard deviations is ray length
                    dx=2*self.regressionMean0
                    #end point is start point plus search length
                    x1=x0+dx

                #midrange
                elif reg-self.regressionMean1<2.5:
                    reg=reg-self.regressionMean1

                    dx=2*self.regressionMean1
                    x0=reg-self.regressionMean1
                    x1=x0+dx
                #far
                else:
                    reg=reg-self.regressionMean2
                    dx=2*self.regressionMean2

                    x0=reg-self.regressionMean2
                    x1=x0+dx


            idx= max(int(math.ceil(dx/self.gridResolution)),int(math.ceil(dy/self.gridResolution)),int(math.ceil(dz/self.voxelResolution)))


            #print (dx,dy)

            #print idx

            #sx = -1 if x0 > x1 else 1
            #sy = -1 if y0 > y1 else 1

            ddx=-(x0-x1)/idx
            ddy=-(y0-y1)/idx
            ddz= -(z0-z1)/idx

            for i in xrange(idx):

                points.append((x0+i*ddx,y0+i*ddy,z0+i*ddz))

            #print points

            pointcoords=[]

            for e in points:
                if (e[0]>=0 and e[1]>=0) and (e[0]<=self.mapLength and e[1]<=self.mapWidth and e[2]>=0):
                    pointcoords.append((int(e[0]/self.gridResolution),int(e[1]/self.gridResolution),int(e[2]/self.voxelResolution)))

            #get unique values
            pointcoords=list(set(pointcoords))

            #sort list based on first value then second
            #sortedpointcoords.append(sorted(pointcoords,key=lambda x: (x[0], x[1])))
            cellcoords.append(pointcoords)

            #print "----------------------"
            #print (x0,y0,z0)
            #print (dx,dy,dz)
            #print "----------------------"


        return cellcoords

    def computeImage(self,image):

        windowCenters=[]

        hogData=[]
        regData=[]

        imageHeight = image.shape[0]
        imageWidth = image.shape[1]

        #print (imageWidth,imageHeight)
        #Check that the Window Parameters match the image Dimensions else raise an error quit
        if (imageWidth >= self.hog.winWidth) and ((imageWidth - self.hog.winWidth) % self.hog.winStrideX == 0) and (imageHeight >= self.hog.winHeight) and ((imageHeight - self.hog.winHeight) % self.hog.winStrideY == 0):
            for winY in xrange((imageHeight - self.hog.winHeight) / self.hog.winStrideY + 1):
                for winX in xrange((imageWidth - self.hog.winWidth) / self.hog.winStrideX + 1):

                    startY = winY * self.hog.winStrideY
                    endY = winY * self.hog.winStrideY + self.hog.winHeight
                    startX = winX * self.hog.winStrideX
                    endX = winX * self.hog.winStrideX + self.hog.winWidth

                    windowCenters.append((startX+(endX-startX)/2,startY+(endY-startY)/2))
                    hog=self.hog.computeBlocks(image[startY:endY,startX:endX])
                    hogData.append(hog)
        else:
            #raise CustomException('Justimage Shape - Window Parameter mismatch: Window Parameters adhere to shape values: following conditional statements are in chcked order:')
            print (imageWidth >= self.hog.winWidth)
            print ((imageWidth - self.hog.winWidth) % self.hog.winStrideX == 0)
            print (imageHeight >= self.hog.winHeight)
            print ((imageHeight - self.hog.winHeight) % self.hog.winStrideY)

        return (windowCenters,hogData)


    def evaluate(self):

        return np.unravel_index(self.probabilityMap.argmax(), self.probabilityMap.shape)



