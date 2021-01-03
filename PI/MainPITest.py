import sys, serial, os
import numpy as np
from time import sleep
from KeyIn import KBHit
from datetime import datetime
import argparse
import math
import time
import atexit
# from vision import CameraHandler
from vision2 import ProcessOutput as CameraHandler
from picamera import PiCamera

from PIL import Image


class Position():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return "X={}, Y={}, Theta={}".format(self.x,self.y,self.theta)

    def __add__(self,other):
        if isinstance(other, int) or isinstance(other, float):
            x = self.x + other
            y = self.y + other
            theta = self.theta + other
        if isinstance(other, Position):
            x = self.x + other.x
            y = self.y + other.y
            theta = self.theta + other.theta

        return self.__class__(x, y, theta)

    def __sub__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            x = self.x - other
            y = self.y - other
            theta = self.theta

        if isinstance(other, Position):
            x = self.x - other.x
            y = self.y - other.y
            theta = self.theta - other.theta

        return self.__class__(x, y, theta)

    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            x = self.x * other
            y = self.y * other
            theta = self.theta * other
        if isinstance(other, Position):
            x = self.x * other.x
            y = self.y * other.y
            theta = self.theta * other.theta

        return self.__class__(x, y,theta)

    def __radd__(self, other):
        return self.__add__(other)

    def __rsub__(self, other):
        # x - y != y - x
        if isinstance(other, float) or isinstance(other, int):
            x = other - self.x
            y = other - self.y
            theta =  self.theta

        return self.__class__(x, y, theta)
    def __rmul__(self, other):
        return self.__mul__(other)



class AV:
    def __init__(self,serial_port,serial_rate):
        self.ser = serial.Serial(serial_port, serial_rate, timeout=1)
        self.ser.flushInput()

        # sleep(2)

        self.K = 0.3
        self.B = 3

        self.visionK = 0.005
        self.visionB = 0.01

        self.vision = CameraHandler(ROITop=160,ROIHeight= 70)
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32

        self.camera.start_preview()
        time.sleep(2)
        self.camera.start_recording(self.vision, format='mjpeg')
        # self.vision.startRecording()
        self.frameNumber = 0
        self.visionLastError = 0

        self.wheelDiameter = 7
        self.wheelBase = 17
        self.wheelCircumference = 7 * math.pi
        self.wheelDepth = 0.8
        self.rMaxTick = 8
        self.lMaxTick = 8
        self.rDistPerTick = self.wheelCircumference / self.rMaxTick
        self.lDistPerTick = self.wheelCircumference / self.lMaxTick
        self.distYoke = 12.5 # from the center of mass to the front point, yoke

        self.currentRightTick = 0
        self.currentLeftTick = 0
        self.currentRightPWM = 0
        self.currentLeftPWM = 0
        self.egoPos = Position(0,0,0)
        self.deriveYokePos()

        self.goalPos = Position(0,0,0)
        self.pastRef = Position(0,0,0)

        self.lastUpdateTime = time.time()
        self.totalTimePassed = 0
        self.PDHerzTarget = 15

        self.distance = 300

        self.vel = 10

    def deriveYokePos(self):
        self.yokePos = Position(self.egoPos.x + self.distYoke * math.cos(self.egoPos.theta),
                                self.egoPos.y + self.distYoke * math.sin(self.egoPos.theta),
                                self.egoPos.theta)


    def getCircleRef(self,t,vel):
        # r = 250
        # d = t * vel
        # s = (2*math.pi*r)/d
        # theta = s/r
        # x = r * math.cos(theta)
        # y = r * math.sin(theta)
        # return Position(x,y,0)
        r = 30
        theta = vel * t / (r) - 0.5 * math.pi
        x = r * math.cos(theta)
        y = r * math.sin(theta) + r
        return Position(x + self.distYoke, y, 0)

    def getRef(self,t,vel):
        dist = t*vel
        d = 100
        return Position(dist + self.distYoke, 0, 0)
        if dist < d:
            return Position(dist+self.distYoke,0,0)
        else:
            return Position(d+self.distYoke,(t-(d/vel))*vel,0)

    def __writeToSerial(self, data):
        # print ('Sending data to arduino ', data)
        self.ser.write((str(data) + ' ').encode())

    def __readLineSerial(self):
        line = str(self.ser.readline())
        return line.strip()

    def getArduinoUpdate(self):
        self.__writeToSerial(600)
        line = self.__readLineSerial()
        data = line.split(",")
        print("Right Tick={}, Left Tick={}, Distance={}cm".format(data[0], data[1], data[2]))
        return int(data[0]), int(data[1]), int(data[2])

    def setWheelsPWM(self,rightWheel,leftWheel):

        rightWheel = int(rightWheel)
        leftWheel = int(leftWheel)

        if rightWheel > 400:
            rightWheel = 400
        if rightWheel < -400:
            rightWheel = -400

        if leftWheel > 400:
            leftWheel = 400
        if leftWheel < -400:
            leftWheel = -400

        PWM_Code = 100000000
        rd = 1 if rightWheel >= 0 else 2
        ld = 1 if leftWheel >= 0 else 2

        R_PWM_Current = ((rd * 1000) + abs(rightWheel))
        L_PWM_Current = ((ld * 1000) + abs(leftWheel) )* 10000
        # print("right wheel={},left wheel={}".format(rightWheel,leftWheel))

        PWM_Current = PWM_Code + R_PWM_Current + L_PWM_Current
        self.currentRightPWM = rightWheel
        self.currentLeftPWM = leftWheel
        self.__writeToSerial(PWM_Current)

    def reachGoal(self):
        distFromGoal = self.goalPos - self.yokePos
        threshold = 20
        if abs(distFromGoal.x) < threshold and abs(distFromGoal.y) < threshold:
            self.stop()


    def update(self):

        now = time.time()

        sleepTime = self.lastUpdateTime + (1.0 / self.PDHerzTarget) - now
        if sleepTime > 0:
            sleep(sleepTime)
        
        now = time.time()
        deltaTime = now - self.lastUpdateTime

        self.totalTimePassed += deltaTime
        self.lastUpdateTime = now
        
        
        rTick, lTick, self.distance = self.getArduinoUpdate()
        deltaSL = (lTick - self.currentLeftTick) * self.lDistPerTick
        deltaSR = (rTick - self.currentRightTick) * self.rDistPerTick
        deltaX = (deltaSR + deltaSL) / 2
        deltaTheta = math.atan2((deltaSR - deltaSL) / 2, self.wheelBase / 2)

        deltaXR = deltaX * math.cos(self.egoPos.theta + deltaTheta)
        deltaYR = deltaX * math.sin(self.egoPos.theta + deltaTheta)
        #
        print("==============================")
        # # print ("TIME={}".format(self.totalTimePassed))
        print ("DaltaTIME={}".format(deltaTime))
        self.egoPos += Position(deltaXR,deltaYR,deltaTheta)
        # #
        self.deriveYokePos()
        # # print("yoke after: {}".format(self.yokePos))
        #
        self.currentLeftTick = lTick
        self.currentRightTick = rTick
        #
        # xDot = deltaXR / deltaTime
        # yDot = deltaYR / deltaTime
        # thetaDot = deltaTheta / deltaTime
        # yokeDotPos = Position(xDot + self.distYoke * -math.sin(self.egoPos.theta) * thetaDot,
        #                       yDot + self.distYoke * math.cos(self.egoPos.theta) * thetaDot,
        #                       thetaDot)
        #
        # refPos = self.getRef(self.totalTimePassed,self.vel)
        # # refPos = self.getCircleRef(self.totalTimePassed,self.vel)
        # # print ("REF={}".format(refPos))
        #
        # errorP = self.yokePos - refPos
        # errorD = Position(yokeDotPos.x - ((refPos.x - self.pastRef.x)/deltaTime),
        #                   yokeDotPos.y - ((refPos.y - self.pastRef.y)/deltaTime),
        #                   0)
        #
        # # print("ErrorP={}".format(errorP))
        # # print("errorD={}".format(errorD))
        # self.pastRef = refPos


        # F_PD = (-1 * (self.K * errorP) )+ (-1 * (self.B * errorD))
        #
        # # print ("F_PD= {}".format(F_PD))
        #
        # errorAngleFromBase = math.atan2(errorP.y, errorP.x)
        # # eTheta =  errorAngleFromBase - self.egoPos.theta
        # eTheta =  self.egoPos.theta
        #
        # # print("theta= {}, error theta= {}, projection angle= {}".format(self.egoPos.theta, errorAngleFromBase, eTheta))
        #
        # x_hat = math.cos(eTheta)
        # y_hat = math.sin(eTheta)
        #
        # # print("xhat={}, yhat={}".format(x_hat,y_hat))
        #
        # F_trans = F_PD.x * x_hat + F_PD.y * y_hat
        # F_rot =  F_PD.x * (y_hat) - F_PD.y * ( x_hat)

        # print ("F_trans= {}".format(F_trans))
        # print ("F_rot= {}".format(F_rot))


        # rot_w = 0.8
        # self.currentRightPWM += F_trans - (F_rot * rot_w)
        # self.currentLeftPWM += F_trans + (F_rot * rot_w)

        ################### VISION ##############################
        redline = 0
        visionError = self.vision.error
        if visionError != None and (visionError != 10000):
            visionErrorROC = (visionError - self.visionLastError ) / deltaTime
            F_PD_vision = (-1 * (self.visionK * visionError)) + (-1 * (self.visionB * visionErrorROC))
            print("F_PD_vision= {}, visionROC= {}, error= {}".format(F_PD_vision,visionErrorROC, visionError))

            self.visionLastError = visionError

            # cumulative error correction (disabled for the moment)
            self.currentRightPWM += -F_PD_vision
            self.currentLeftPWM +=  F_PD_vision

            # absolute error correction (not general, but holds speed better)
            # self.currentRightPWM = 145 + F_PD_vision
            # self.currentLeftPWM = 134 - F_PD_vision

            print("frame={}, error={}, gtCenter={}, rightLane={}, leftLane={}".format(self.frameNumber, visionError, self.vision.gtCenter,
                                                                                  self.vision.rightLane, self.vision.leftLane))
            Image.fromarray(self.vision.markedFrame).save("../sampleimg/test/{}.jpg".format(self.frameNumber))
            self.frameNumber+=1

            redline = self.vision.redline
            print ("RedLine ======= {}".format(redline))
        ######################CHECK FOR SPEED#####################

        speed = deltaX / deltaTime
        print ('deltaX={} Speed={}'.format(deltaX,speed))
        if speed < self.vel:
            self.currentLeftPWM += 1
            self.currentRightPWM += 1

        if speed > self.vel+ 10:
            self.currentLeftPWM -= 1
            self.currentRightPWM -= 1


        if self.distance != 0 and self.distance < 30:
            self.__writeToSerial(500)
            return

        # if redline > 200:
        #     self.__writeToSerial(500)
        #     return

        self.setWheelsPWM(self.currentRightPWM, self.currentLeftPWM)

        # self.reachGoal()

    def stop(self):
        print("Stop")
        self.__writeToSerial(500)
        self.vision.done = True
        self.camera.stop_recording()
        # self.vision.stopRecording()
        sleep(1)

    def rightPWMtoVel(self,PWM):
        slope = 0.17250446
        intercept = -14.93387235
        return PWM * slope + intercept

    def rightVeltoPWM(self,vel):
        slope = 5.75863643
        intercept = 87.713437
        return int(vel * slope + intercept)

    def leftPWMtoVel(self,PWM):
        slope = 0.17139429
        intercept = -13.02985714
        return PWM * slope + intercept

    def leftVeltoPWM(self,vel):
        slope =  5.82088733
        intercept = 76.42862371
        return int(vel * slope + intercept)

def main():
    parser = argparse.ArgumentParser(description='AVDocky')
    parser.add_argument("-p", help='Port of the serial' ,required=True)
    parser.add_argument("-R", help='Rate of the serial' , default=115200)
    parser.add_argument("-r", type=int,help='Right wheel PWM' , default=400)
    parser.add_argument("-l", type=int,help='Left wheel PWM' , default=400)
    parser.add_argument("-speed", type=float,help='Speed of going straight cm/s' , default=10)

    parser.add_argument("-x", type=float,help="X position", default=3000)
    parser.add_argument("-y", type=float,help="Y position", default=0)
    parser.add_argument("-th", type=float,help="Theta", default=0)
    args = parser.parse_args()

    av = AV(args.p,args.R)
    av.vel = args.speed
    atexit.register(av.stop)
    av.goalPos = Position(args.x,args.y,args.th)

    sleep(1)
    # av.stop()
    # exit()
    kb = KBHit()

    av.lastUpdateTime = time.time()
    av.setWheelsPWM(200, 200)
    sleep(1.68)
    # av.setWheelsPWM(180, 115)
    # sleep(5.5)
    av.setWheelsPWM(90, 175)
    sleep(2.0)
    av.stop()


    try:
        pass

    #     while(True):
    #         print(time.time()-start_time)
    #         now = time.time()
    #         sleep(0.05)

    #         if (time.time() - start_time > 5.2):
    #             av.stop()
    #             return
    #         pass
            # if av.yokePos.x >= 200:
            #     print("Destination")
            #     av.stop()
            #     exit(0)

            # sleep(0.1)
            # av.update()
            # print ("Current Values ====> Pos:{}".format(av.egoPos))

            # if av.distance < 30:
            #     print("DISTANCE")
            #     av.stop()
            #     exit(0)
    except KeyboardInterrupt:
        print ("CTRL+C pressed")
        av.stop()

    sleep(2)


if __name__ == '__main__':
    main()
