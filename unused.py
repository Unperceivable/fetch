
#SomeCamera Functions

LOWER_BLUE=np.array([100,0,0],dtype=np.uint8)
UPPER_BLUE=np.array([180,255,255],dtype=np.uint8)

		self.extractColor=False


			if self.extractColor==True:
				cv2.imshow("Extracted", self.ExtractColor(numpyImage,LOWER_BLUE,UPPER_BLUE))

			if self.edgeDetection==True:
				cv2.imshow("Edges", cv2.Canny(numpyImage,100,200))

	#Takes 2 np arrays of color bounds to track
	def ExtractColor(self,frame,lower,upper):
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask=cv2.inRange(hsv,lower,upper)
		extracted=cv2.bitwise_and(frame,frame,mask=mask)			

		return extracted


	def StartExtractColor(self):
		self.extractColor=True


