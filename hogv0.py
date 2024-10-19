import numpy as np
import cv2
import math

class HOG:

	def __init__(self,winSize=(64,64),
			winStride=(32,32),
			blockSize=(32,32),
			blockStride=(16,16),
			cellSize=(16,16),
			nbins=9,
			scaleFactor=0.5,
			vizFactor=1):
		#Size=(height,width)
		#Initialize HOG Window Parameters
		self.winWidth=winSize[1]
		self.winHeight=winSize[0]
		self.winStrideX=winStride[1]
		self.winStrideY=winStride[0]
		
		#Initialize HOG Block Parameters
		self.blockWidth=blockSize[1]
		self.blockHeight=blockSize[0]
		self.blockStrideX=blockStride[1]
		self.blockStrideY=blockStride[0]

		#Initialize HOG Cell Parameters
		self.cellWidth=cellSize[1]
		self.cellHeight=cellSize[0]

		#Initialize HOG Misc Parameters
		self.nbins=nbins
		self.scaleFactor=scaleFactor
		self.vizFactor=vizFactor

	def ComputeHOGBlocks(self,image):

		# .shape = (height,width)
		imageHeight=image.shape[0]	
		imageWidth=image.shape[1]

		gx = cv2.Sobel(image, cv2.CV_32F, 1, 0)
		gy = cv2.Sobel(image, cv2.CV_32F, 0, 1)
		mag, ang = cv2.cartToPolar(gx, gy)
		bins = np.int32(self.nbins*ang/(2*np.pi))
		

		binBlockValues=()
		magBlockValues=()

		#Check that the Window Parameters match the image Dimensions else raise an error quit
		if (imageWidth>=self.winWidth) and ((imageWidth-self.winWidth) % self.winStrideX==0) and (imageHeight>=self.winHeight) and ((imageHeight-self.winHeight) % self.winStrideY==0):
			
			#Check that Block Parameters match the Window Parameters else raise an error and quit
			if (self.winWidth % self.blockWidth == 0) and (self.blockWidth % self.cellWidth == 0) and (self.blockStrideX % self.cellWidth==0) and (self.winHeight % self.blockHeight == 0) and (self.blockHeight % self.cellHeight == 0) and (self.blockStrideY % self.cellHeight==0):
				#Check that Cell Parameters match the Block Parameters else raise and error and quit
				if (self.blockWidth % self.cellWidth == 0) and (self.blockHeight % self.cellHeight ==0):
					
					#For all windows that fit in the image
					for winY in xrange((imageHeight-self.winHeight)/self.winStrideY+1):
						for winX in xrange((imageWidth-self.winWidth)/self.winStrideX+1):
							#print (winY,winX)	
							#For all blocks in that window							
							for blockY in xrange((self.winHeight-self.blockHeight)/self.blockStrideY+1):
								for blockX in xrange((self.winWidth-self.blockWidth)/self.blockStrideX+1):
									#print (blockY,blockX)	
								
									startX = winX*self.winStrideX + blockX*self.blockStrideX
									endX= winX*self.winStrideX + blockX*self.blockStrideX + self.blockWidth
									startY= winY*self.winStrideY + blockY*self.blockStrideY
									endY= winY*self.winStrideY + blockY*self.blockStrideY + self.blockHeight
									binBlockValues+=(bins[startY:endY,startX:endX],)
									magBlockValues+=(mag[startY:endY,startX:endX],)

									#print (startY,endY,startX,endX)
									
				else:
					#raise CustomException('CellSize - blockSize mismatch: cellSize must be factors of corresponding shape values: following conditional statements are in checked order:')
					print (self.blockWidth % self.cellWidth == 0)
					print (self.blockHeight % self.cellHeight ==0)
					return None
			else:
				#raise CustomException('Block Parameters - Window/Cell Parameter mismatch: Block Parameters must adhere to window and cell values: following conditional statements are in checked order:')	
				print (self.winWidth % self.blockWidth == 0)
				print (self.blockWidth % self.cellWidth == 0)
				print (self.blockStrideX % self.cellWidth==0) 
				print (self.winHeight % self.blockHeight == 0)
				print(self.blockHeight % self.cellHeight == 0)
				print (self.blockStrideY % self.cellHeight==0)
				return None

		else:
			#raise CustomException('image Shape - Window Parameter mismatch: Window Parameters adhere to shape values: following conditional statements are in chcked order:')
			print (imageWidth>=self.winWidth)
			print ((imageWidth-self.winWidth) % self.winStrideX==0)
			print (imageHeight>=self.winHeight)
			print ((imageHeight-self.winHeight) % self.winStrideY==0)			
			return None
		
		print magBlockValues
		hists = [np.bincount(b.ravel(), m.ravel(), self.nbins) for b, m in zip(binBlockValues, magBlockValues)]
				
		hist = np.hstack(hists)

		return hist
	
	#Returns HOG Values for Each Cell
	def ComputeHOGCells(self,image):
		gx = cv2.Sobel(image, cv2.CV_32F, 1, 0)
		gy = cv2.Sobel(image, cv2.CV_32F, 0, 1)
		mag, ang = cv2.cartToPolar(gx, gy)
		bins = np.int32(self.nbins*ang/(2*np.pi))

		# .shape = (height,width)
		imageHeight=image.shape[0]	
		imageWidth=image.shape[1]


		binCellValues=()
		magCellValues=()


		if (imageWidth % self.cellWidth == 0) and (imageHeight % self.cellHeight ==0):
			for cellY in xrange(imageHeight/self.cellHeight):
				for cellX in range(imageWidth/self.cellWidth):	
					startX = cellX*self.cellWidth 
					endX=cellX*self.cellWidth + self.cellWidth
					startY=cellY*self.cellHeight
					endY=cellY*self.cellHeight + self.cellHeight
	
					binCellValues+=(bins[startY:endY,startX:endX],)
					magCellValues+=(mag[startY:endY,startX:endX],)
							
					
		else:
			raise CustomException('image Shape - cellSize mismatch: cellSize must be factors of corresponding shape values')
			return None

		#If all magCellValues for first Bins are 0 bincount throw an arrow, so we must catch and adjust the value to make it negligable	
	
		hists = [np.bincount(b.ravel(),m.ravel(), self.nbins) for b, m in zip(binCellValues, magCellValues)]
	
		hist = np.hstack(hists)
		#print hists
		#print len(hists)
		#f=np.array([])
		#for h in hist:
		#	np.concatenate(f,h/sum(h))
		return hist
		#return f
		
		
	def computeBlocks(self,image):
		gx = cv2.Sobel(image, cv2.CV_32F, 1, 0)
		gy = cv2.Sobel(image, cv2.CV_32F, 0, 1)
		mag, ang = cv2.cartToPolar(gx, gy)
		bins = np.int32(self.nbins*ang/(2*np.pi))

		# .shape = (height,width)
		imageHeight=image.shape[0]	
		imageWidth=image.shape[1]
		#print (imageWidth,imageHeight)

		binBlockValues=()
		magBlockValues=()
		#Check that Block Parameters match the Window Parameters else raise an error and quit
		if (self.winWidth % self.blockWidth == 0) and (self.blockWidth % self.cellWidth == 0) and (self.blockStrideX % self.cellWidth==0) and (self.winHeight % self.blockHeight == 0) and (self.blockHeight % self.cellHeight == 0) and (self.blockStrideY % self.cellHeight==0):
			for blockY in xrange((imageHeight-self.blockHeight)/self.blockStrideY+1):
				for blockX in xrange((imageWidth-self.blockWidth)/self.blockStrideX+1):	
					startX = blockX*self.blockStrideX
					endX=blockX*self.blockStrideX + self.blockWidth

					startY=blockY*self.blockStrideY
					endY=blockY*self.blockStrideY + self.blockHeight

					#print "Block: "+str((blockY,blockX))
					binBlockValues+=(bins[startY:endY,startX:endX],)
					magBlockValues+=(mag[startY:endY,startX:endX],)

		else:
			#raise CustomException('Block Parameters - Window/Cell Parameter mismatch: Block Parameters must adhere to window and cell values: following conditional statements are in checked order:')	
			print (self.winWidth % self.blockWidth == 0)
			print (self.blockWidth % self.cellWidth == 0)
			print (self.blockStrideX % self.cellWidth==0) 
			print (self.winHeight % self.blockHeight == 0)
			print(self.blockHeight % self.cellHeight == 0)
			print (self.blockStrideY % self.cellHeight==0)
			return None							
					

		#If all magCellValues for first Bins are 0 bincount throw an arrow, so we must catch and adjust the value to make it negligable	
	
		hists = [np.bincount(b.ravel(),m.ravel(), self.nbins) for b, m in zip(binBlockValues, magBlockValues)]
		#print hists
		blockHist=[]
		for h in hists:
			if sum(h):		
				blockHist.append(h/sum(h))		
			else:
				blockHist.append(h)
		#print blockHist
		hist = np.hstack(blockHist)
		#print hist
		return hist

		

	def visualizeHOG(self,image,hist):
		# .shape = (height,width)
		imageHeight=image.shape[0]	
		imageWidth=image.shape[1]
		#normalize histogram values
		hist/=max(hist)	
		
		binRange =  math.pi/self.nbins
		#bin vector legnth should be as long as the root of the cell dimensions
		maxVecLen= 0.5*( (self.cellWidth)**2 + (self.cellHeight)**2 )**0.5
		currbin=0

		visualization=cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

		if imageHeight % self.cellHeight == 0 and imageWidth % self.cellWidth == 0:
			for cellY in xrange(imageHeight/self.cellHeight):
				for cellX in xrange(imageWidth/self.cellWidth):
					for nbin in xrange(self.nbins):
						 
						#print (cellY,cellX)					
					
						currRad = float(nbin*binRange + binRange/2)	
						dirVecX = float(math.cos(currRad))
						dirVecY = float(math.sin(currRad))

						drawX = int(cellX*self.cellWidth)
						drawY = int(cellY*self.cellHeight)

						mx = int(drawX + self.cellWidth/2)
						my = int(drawY + self.cellHeight/2)

						cv2.rectangle(visualization,
								(drawX*self.vizFactor,drawY*self.vizFactor),
								(drawX+self.cellWidth*self.vizFactor,drawY+self.cellHeight*self.vizFactor),
								(100,100,100),
								1)



						x1 = float(mx + dirVecX*hist[currbin] * maxVecLen * self.vizFactor)
						y1 = float(my + dirVecY*hist[currbin] * maxVecLen * self.vizFactor)
						x2 = float(mx - dirVecX*hist[currbin] * maxVecLen * self.vizFactor)
						y2 = float(my - dirVecY*hist[currbin] * maxVecLen * self.vizFactor)

						cv2.line(visualization,
							(int(x1*self.vizFactor),int(y1*self.vizFactor)),
							(int(x2*self.vizFactor),int(y2*self.vizFactor)),
							(0,0,255),
							1)

						currbin+=1
		else:
			raise CustomException('image Shape - cellSize mismatch: cellSize must be factors of corresponding shape values')
			return None

		return visualization

	def CompareHOG(self,hist1,hist2,l=1.0):

		if len(hist1)==len(hist2):
						
			magnitudeH1=0
			magnitudeH2=0
			for h1, h2 in zip(hist1,hist2):
				magnitudeH1+= h1**l
				magnitudeH2+= h2**l
			return abs(magnitudeH1-magnitudeH2)**(1/l)
			

		else:
			#raise CustomException('Histogram size mismatch: Histrograms compared must has identical size')
			print str(len(hist1)) + "!=" + str(len(hist2))
			return None
	

#raw_img = cv2.imread('/home/constantin/Pictures/Screenshot from 2014-11-28 13:31:05.png',1)

#raw_img = cv2.resize(raw_img,(64,128))
#img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2GRAY)

'''hog=HOG(winSize=(64,128),
		winStride=(32,32),
		blockSize=(64,64),
		blockStride=(32,32),
		cellSize=(32,32),
		nbins=9,
		scaleFactor=1,
		vizFactor=1)
'''
#h=hog.computeBlocks(img)
#h=hog.ComputeHOGCells(img)
#print h.shape
#print hist[0].shape
#print hist[1].shape
#print hist[2].shape
#print hist[3].shape

#hist2=hog.ComputeHOGBlocks(img)
#print len(hist2)




#cv2.imshow("HOGImage",hog.visualizeHOG(img,hist))
#cv2.waitKey()




