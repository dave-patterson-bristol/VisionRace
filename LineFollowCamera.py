import os
import pdb
import socket
import sys
import time

import cv2
import numpy as np

from Image import *
from Utils import *
from XboxController import *

def nothing(x):
    pass

class LineFollowCamera:

    class PPMchannels():
        Throttle = 1
        Yaw = 4
        Roll = 2
        Pitch = 3
        Aux1 = 5
        Aux2 = 6

    def __init__(self):

        self.comPort = serial.Serial('\\.\COM8')
        
        self.throttle = 1000
        self.yaw = 1500
        self.roll = 1500
        self.pitch = 1500
        self.aux1 = 1000
        self.aux2 = 1000

        self.armed = 0
        self.rangefinder = 0
        self.followMode = 0
        self.turnMode = 0
        self.killSwitch = 0
        self.rate = 1
        self.throttlemid = 0
        self.trimtoggle = 0
        self.yawtrim = 0
        self.pitchtrim = 0
        self.rolltrim = 0
        self.leftTrigger = 0
        self.turnRate = 0
        self.turnRoll = 0
        self.turnDuration = 0

        direction = 0
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        Images=[]
        N_SLICES = 6

        for q in range(N_SLICES):
            Images.append(Image())

        xboxCont = XboxController(self.newControlCallBack, deadzone = 30, scale = 100, invertYAxis = True)
        xboxCont.start()
        print("xbox controller running")

        cap = cv2.VideoCapture(0)
        cv2.namedWindow('params',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('params', 800,800)

        hh='Hue High'
        hl='Hue Low'
        sh='Saturation High'
        sl='Saturation Low'
        vh='Value High'
        vl='Value Low'
        self.paramPitch='Pitch'
        self.paramAngleRate='Angle rate'
        self.paramDeviationRate='Deviation rate'
        self.paramTurnTime='Turn duration'
        self.paramTurnRoll='Turn roll'
        self.paramTurnYaw='Turn yaw'


        cv2.createTrackbar(hl, 'params',0,179,nothing)
        cv2.createTrackbar(hh, 'params',179,179,nothing)
        cv2.createTrackbar(sl, 'params',0,255,nothing)
        cv2.createTrackbar(sh, 'params',255,255,nothing)
        cv2.createTrackbar(vl, 'params',0,255,nothing)
        cv2.createTrackbar(vh, 'params',255,255,nothing)
        cv2.createTrackbar(self.paramPitch, 'params', 0, 255, nothing)
        cv2.createTrackbar(self.paramAngleRate, 'params', 0, 100,nothing)
        cv2.createTrackbar(self.paramDeviationRate, 'params',0,100,nothing)
        cv2.createTrackbar(self.paramTurnTime, 'params',0,10000,nothing)
        cv2.createTrackbar(self.paramTurnRoll, 'params',0,255,nothing)
        cv2.createTrackbar(self.paramTurnYaw, 'params',0,255,nothing)

        while(1):
            _,frame=cap.read()
            frame=cv2.GaussianBlur(frame,(5,5),0)
            #convert to HSV from BGR
            #img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            img = frame
            direction = 0
            img = RemoveBackground(img, True, cv2)
            if img is not None:
                t1 = time.clock()
# Chop the image into N horizontal stripes                    
                SlicePart(img, Images, N_SLICES)
                leadDir = Images[1].deviation
                trailingDir = Images[3].deviation

                angleCorrection = (leadDir - trailingDir) * (cv2.getTrackbarPos(self.paramAngleRate, 'params') / 10)
                deviationCorrection = trailingDir * (cv2.getTrackbarPos(self.paramDeviationRate, 'params') / 10)
                
                direction = (angleCorrection + deviationCorrection)

                if self.followMode == 1:
                    if self.turnMode !=1:
                        self.pitch = 1500 + self.pitchtrim - int(cv2.getTrackbarPos(self.paramPitch, 'params'))
                        self.updateControls(self.PPMchannels.Pitch, self.pitch)
                        self.yaw = 1500 + self.yawtrim - int(direction)
                        self.updateControls(self.PPMchannels.Yaw, self.yaw)

                fm = RepackImages(Images)
                t2 = time.clock()
                cv2.putText(fm, "Time: " + str((t2-t1)*1000)[0:4] + " ms", (10, 470),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                if self.armed == 1:
                    cv2.putText(fm, "ARMED", (10, 20),
                                font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                if self.rangefinder == 1:
                    cv2.putText(fm, "RANGEFINDER", (10, 50),
                                font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                if self.followMode == 1:
                    cv2.putText(fm, "FOLLOW", (400, 20),
                                font, 0.5, (0, 0, 255), 1,cv2.LINE_AA)
                cv2.putText(fm, "Trottle: " + str(self.throttle), (10, 390),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(fm, "Yaw: " + str(self.yaw), (10, 410),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(fm, "Pitch: " + str(self.pitch), (10, 430),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(fm, "Roll: " + str(self.roll), (10, 450),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(fm, "Angle correction: " + str(angleCorrection), (400, 430),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(fm, "Deviation correction: " + str(deviationCorrection), (400, 450),
                            font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

                cv2.imshow("Vision Race", fm)
# Push the navigation instruction
# Needs to send a "no lock" message if there isn't a clear nav decision
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    def newCalculateThrottle(self, throttlemid, value):
        if value <= 0:
            return int((value/100 +1) * throttlemid) + 1000
        else:
            return int((1000 - throttlemid) * (value / 100)) + throttlemid + 1000
    
    def updateControls(self, channel, value):
        self.comPort.write("{},{},".format(channel,value).encode())
        if (channel == self.PPMchannels.Aux1): self.aux1 = value
        if (channel == self.PPMchannels.Aux2): self.aux2 = value
        if (channel == self.PPMchannels.Throttle): self.throttle = value
        if (channel == self.PPMchannels.Yaw): self.yaw = value
        if (channel == self.PPMchannels.Pitch): self.pitch = value
        if (channel == self.PPMchannels.Roll): self.roll = value

    #generic call back
    def newControlCallBack(self, xboxControlId, value):
        print ("Control Id = {}, Value = {}".format(xboxControlId, value))
        #Throttle
        if xboxControlId == 1:
            self.throttle = self.newCalculateThrottle(self.throttlemid, value)
            self.updateControls(self.PPMchannels.Throttle, self.throttle)
        # Trim throttle mid
        if xboxControlId == 17:
            if value[1] == 1:
                if self.throttlemid < 700:
                    self.throttlemid += 50
                    self.throttle = self.newCalculateThrottle(self.throttlemid, 0)
                    self.updateControls(self.PPMchannels.Throttle,self.throttle)                    
            if value[1] == -1:
                if self.throttlemid >= 100:
                    self.throttlemid -= 50
                    self.throttle = self.newCalculateThrottle(self.throttlemid, 0)
                    self.updateControls(self.PPMchannels.Throttle,self.throttle)
            if value[0] == 1:
                self.yawtrim += 3
                self.yaw = 1500 + self.yawtrim
                self.updateControls(self.PPMchannels.Yaw, self.yaw)
            if value[0] == -1:
                self.yawtrim -= 3
                self.yaw = 1500 + self.yawtrim
                self.updateControls(self.PPMchannels.Yaw, self.yaw)
        #Yaw
        if xboxControlId == 0: 
            self.yaw = 1500 + yawtrim + int(value * rate)
            self.updateControls(self.PPMchannels.Yaw, self.yaw)
       
        if xboxControlId == 15:
            self.trimtoggle = value

        if self.trimtoggle == 1:
            #Roll
            if xboxControlId == 4: 
                self.rolltrim = int((value-50)/2)
                self.roll = 1500 - self.rolltrim
                self.updateControls(self.PPMchannels.Roll, self.rolltrim)
            #Pitch
            if xboxControlId == 3:
                self.pitchtrim = int(value/2)
                self.pitch = 1500 - self.pitchtrim
                self.updateControls(self.PPMchannels.Pitch, self.pitch)
        else:
            #Roll
            if xboxControlId == 4:
                self.roll = (1500 - self.rolltrim) - int(((value-50) *2) * self.rate)
                self.updateControls(self.PPMchannels.Roll, self.roll)
            #Pitch
            if xboxControlId == 3:
                self.pitch = (1500 - self.pitchtrim) - int(value * self.rate)
                self.updateControls(self.PPMchannels.Pitch, self.pitch)
       
        #Toggle aux1 (Armed)
        if xboxControlId == 10:
            if value == 1:
                winsound.Beep(639, 100)
                if self.armed == 1:
                    self.armed = 0
                else: self.armed = 1
                self.updateControls(self.PPMchannels.Aux1, 1100 + int(self.armed * 800))
       
        #Toggle aux2 (Rangefinder)
        if xboxControlId == 11:
            if value == 1:
                winsound.Beep(639, 100)
                if self.rangefinder == 1:
                    self.rangefinder = 0
                else: self.rangefinder = 1
                self.updateControls(self.PPMchannels.Aux2, 1100 + int(self.rangefinder * 800))
  
        #Toggle follow mode - Green button
        if xboxControlId == 6:
            if value == 1:
                winsound.Beep(639,100)
                if self.followMode == 0:
                    self.followMode = 1
                    self.updateControls(self.PPMchannels.Roll, 1500 + self.rolltrim)
                else: 
                    self.followMode = 0
                    self.updateControls(self.PPMchannels.Yaw, 1500 + self.yawtrim)
                    self.updateControls(self.PPMchannels.Pitch, 1500 + self.pitchtrim)
    # scroll manual rate - Amber button
        if xboxControlId == 9:
            
            if value == 1:
                self.rate += 1
                if self.rate == 6: self.rate = 1
                winsound.Beep(610 + int((self.rate * 20)), 100)
            
    # Fire kill switch - Red button
        if xboxControlId == 7:
            winsound.Beep(639, 100)
            # streight
            self.updateControls(self.PPMchannels.Yaw, 1500 + self.yawtrim)
            # kill the rangefinder
            self.updateControls(self.PPMchannels.Aux2, 1000)
            # brakes on
            reverse = 1500 + self.pitchtrim - (self.pitch - (1500 + self.pitchtrim))
            self.pitch = reverse
            self.updateControls(self.PPMchannels.Pitch, self.pitch)
            # dipp throttle for 2.5 seconds.
            self.updateControls(self.PPMchannels.Throttle, self.newCalculateThrottle(self.throttlemid, -20))
            time.sleep(2)
            # disarm the motors
            self.updateControls(self.PPMchannels.Aux1, 1000)
            self.armed = 0
            self.followMode = 0
            self.rangefinder = 0
    # Fire timed turn - Blue button
        if xboxControlId == 8:
            if value == 1:
                self.turnMode = 1
                self.yaw = 1500 + self.yawtrim + cv2.getTrackbarPos(self.paramTurnYaw, 'params')
                self.updateControls(self.PPMchannels.Yaw, self.yaw)
                self.roll = 1500 - self.rolltrim - cv2.getTrackbarPos(self.paramTurnRoll, 'params')
                self.updateControls(self.PPMchannels.Roll, self.roll)
                time.sleep(cv2.getTrackbarPos(self.paramTurnTime, 'params') / 1000 )
    # Reset to level
                self.turnMode = 0
                self.updateControls(self.PPMchannels.Yaw, 1500 + self.yawtrim)
                self.updateControls(self.PPMchannels.Roll, 1500 - self.rolltrim)
        
        if xboxControlId == 12:
            if value == 1:
                winsound.Beep(639,100)
                if self.followMode == 0:
                    self.followMode = 1
                else:
                    self.followMode = 0

if __name__ == '__main__':
    print ("starting")
    try:
        #create class
        lineFollow = LineFollowCamera()
        while lineFollow.running:
            time.sleep(0.1)

    #Ctrl C
    except KeyboardInterrupt:
        print("User cancelled")

    #Error
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
