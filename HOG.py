import cv2
import numpy as np
import matplotlib.image as mpimg

img = cv2.imread('/home/constantin/Pictures/Screenshot from 2014-11-28 13:31:05.png')

hog = cv2.HOGDescriptor()

result=hog.compute(img)

print result
