import numpy as np
import cv2
import time
from Image import *

def SlicePart(im, images, slices):
    height, width = im.shape[:2]
    sl = int(height/slices)
    
    for i in range(slices):
        part = sl*i
        crop_img = im[part:part+sl, 0:width]
        images[i].image = crop_img
        if (i == 1 or i == 3):
            images[i].Process()
    
def RepackImages(images):
    img = images[0].image
    for i in range(len(images)):
        if i == 0:
            img = np.concatenate((img, images[1].image), axis=0)
        if i > 1:
            img = np.concatenate((img, images[i].image), axis=0)
            
    return img

def Center(moments):
    if moments["m00"] == 0:
        return 0
        
    x = int(moments["m10"]/moments["m00"])
    y = int(moments["m01"]/moments["m00"])

    return x, y
    
def RemoveBackground(image, b, cs2):
    up = 100

    # create NumPy arrays from the boundaries
    lower = np.array([0, 0, 0], dtype = "uint8")
    upper = np.array([up, up, up], dtype = "uint8")

    hh='Hue High'
    hl='Hue Low'
    sh='Saturation High'
    sl='Saturation Low'
    vh='Value High'
    vl='Value Low'

   #read trackbar positions for all
    hul=cv2.getTrackbarPos(hl, 'params')
    huh=cv2.getTrackbarPos(hh, 'params')
    sal=cv2.getTrackbarPos(sl, 'params')
    sah=cv2.getTrackbarPos(sh, 'params')
    val=cv2.getTrackbarPos(vl, 'params')
    vah=cv2.getTrackbarPos(vh, 'params')

    #make array for final values
    HSVLOW=np.array([hul,sal,val])
    HSVHIGH=np.array([huh,sah,vah])

    #----------------COLOR SELECTION-------------- (Remove any area that is whiter than 'upper')
    if b == True:
        mask = cv2.inRange(image, HSVLOW, HSVHIGH)
        image = cv2.bitwise_and(image, image, mask = mask)
        image = cv2.bitwise_not(image, image, mask = mask)
        #image = cv2.bitwise_not(image, image)
        image = (255-image)
        return image
    else:
        return image
    #////////////////COLOR SELECTION/////////////
    