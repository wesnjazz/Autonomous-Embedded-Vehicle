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
import graph
import enum
import copy


class AVState(enum.Enum):
    doingRightTurn = 0
    doingLeftTurn = 1
    doingShortStraightLine = 2
    doingLongStraightLine = 3
    usingVision = 4
    waitForGreenLight = 5
    none = 6

    def __lt__(self, other):
        # heappop needs this to be defined
        # the implementation doesn't matter
        return False


class Position():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return "X={}, Y={}, Theta={}".format(self.x, self.y, self.theta)

    def __add__(self, other):
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

        return self.__class__(x, y, theta)

    def __radd__(self, other):
        return self.__add__(other)

    def __rsub__(self, other):
        # x - y != y - x
        if isinstance(other, float) or isinstance(other, int):
            x = other - self.x
            y = other - self.y
            theta = self.theta

        return self.__class__(x, y, theta)

    def __rmul__(self, other):
        return self.__mul__(other)


class AV:
    def __init__(self, serial_port, serial_rate, directions, sequence):
        self.ser = serial.Serial(serial_port, serial_rate, timeout=1)
        self.ser.flushInput()
        self.directions = directions
        self.sequence = sequence
        self.departureNode = -1
        self.arrivalNode = -1
        # sleep(2)
        self.avstate = AVState.none
        print(self.sequence)
        self.K = 1.125
        self.B = 2.5

        # self.visionK = 0.008
        self.visionK = 0.007
        self.visionB = 0.018 # 0.018

        self.vision = CameraHandler(AV=self, ROITop=160, ROIHeight=100)
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
        self.distYoke = 12.5  # from the center of mass to the front point, yoke

        self.traveledDistance = 0
        self.traveledRightTick = 0
        self.traveledLeftTick = 0
        self.currentRightTick = 0
        self.currentLeftTick = 0
        self.currentRightPWM = 0
        self.currentLeftPWM = 0
        self.egoPos = Position(0, 0, 0)
        self.deriveYokePos()

        self.goalPos = Position(0, 0, 0)
        self.pastRef = Position(0, 0, 0)

        self.lastUpdateTime = time.time()
        self.totalTimePassed = 0
        self.PDHerzTarget = 15

        self.distance = 300

        self.vel = 10
        self.nextTurn = None
        self.nextTurnStr = ""

    def deriveYokePos(self):
        self.yokePos = Position(self.egoPos.x + self.distYoke * math.cos(self.egoPos.theta),
                                self.egoPos.y + self.distYoke * math.sin(self.egoPos.theta),
                                self.egoPos.theta)

    def getCircleRef(self, t, vel):
        r = 52
        theta = vel * t / (r) - 0.5 * math.pi
        x = r * math.cos(theta)
        y = r * math.sin(theta) + r
        return Position(x + self.distYoke, y, 0)

    def getRef(self, t, vel):
        dist = t * vel
        return Position(dist + self.distYoke, 0, 0)

    def resetOdometry(self):
        self.egoPos = Position(0, 0, 0)
        self.pastRef = Position(0, 0, 0)
        self.deriveYokePos()
        self.totalTimePassed = -0.15 # there's a small pause at the start, and we can't find the source...

    def getLongStraightLineRef(self, t, vel):
        dist = t * vel
        return Position(dist + self.distYoke, 0, 0)

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
        # print("Right Tick={}, Left Tick={}, Distance={}cm".format(data[0], data[1], data[2]))
        return int(data[0]), int(data[1]), int(data[2])

    def setWheelsPWM(self, rightWheel, leftWheel):

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

        # for test, stop motor
        # leftWheel = 0
        # rightWheel = 0

        PWM_Code = 100000000
        rd = 1 if rightWheel >= 0 else 2
        ld = 1 if leftWheel >= 0 else 2

        R_PWM_Current = ((rd * 1000) + abs(rightWheel))
        L_PWM_Current = ((ld * 1000) + abs(leftWheel)) * 10000
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

    def chooseRightTurn(self):
        sharpTurn = 0
        if self.departureNode == 9 and self.arrivalNode == 1 or \
                self.departureNode == 12 and self.arrivalNode == 9 or \
                self.departureNode == 6 and self.arrivalNode == 3 or \
                self.departureNode == 2 and self.arrivalNode == 4 or \
                self.departureNode == 4 and self.arrivalNode == 11:
            sharpTurn = 1.8
        elif self.departureNode == 8 and self.arrivalNode == 6 or \
                self.departureNode == 3 and self.arrivalNode == 8 or \
                self.departureNode == 11 and self.arrivalNode == 2:
            sharpTurn = 1.7
        return sharpTurn


    def doRightTurn(self, t):
        # self.traveledRightTick = 0
        # self.traveledLeftTick = 0
        self.doShortLineForRight()
        self.avstate = AVState.doingRightTurn
        self.setWheelsPWM(0, 200)
        sleep(t)
        # print("\t\t\tTraveledTicks - L:{} R:{}".format(self.traveledLeftTick, self.traveledRightTick))
        self.avstate = AVState.usingVision

    # def doRightTurnSharp(self):
    #     self.doShortLineForRightSharp()
    #     self.avstate = AVState.doingRightTurn
    #     self.setWheelsPWM(0, 200)
    #     sleep(2.38)
    #     self.avstate = AVState.usingVision

    # def doRightTurnSharpLonger(self):
    #     self.doShortLineForRightSharpLonger()
    #     self.avstate = AVState.doingRightTurn
    #     self.setWheelsPWM(0, 200)
    #     sleep(2.38)
    #     self.avstate = AVState.usingVision

    def doLeftTurn(self):
        # self.K = 1.5
        # self.B = 2.5
        # self.doShortLineForRight()
        self.avstate = AVState.doingLeftTurn
        # self.resetOdometry()
        self.doShortLineForRight()
        self.setWheelsPWM(220, 110)
        sleep(2.55)  # (5.5)
        # sleep(3.0)
        # self.setWheelsPWM(140, 130)  # (180, 130)
        # sleep(0.7)
        self.avstate = AVState.usingVision

    def doShortLineForLeft(self):
        self.avstate = AVState.doingShortStraightLine
        self.setWheelsPWM(210, 170)
        sleep(3.4)

    def doShortLineForRight(self):
        self.avstate = AVState.doingShortStraightLine
        self.setWheelsPWM(220, 190)
        sleep(1.7)

    # def doShortLineForRightSharp(self):
    #     self.avstate = AVState.doingShortStraightLine
    #     self.setWheelsPWM(220, 190)
    #     sleep(1.9)
    #
    # def doShortLineForRightSharpLonger(self):
    #     self.avstate = AVState.doingShortStraightLine
    #     self.setWheelsPWM(220, 190)
    #     sleep(2.2)

    def doShortLineForStraight(self):
        self.avstate = AVState.doingShortStraightLine
        self.resetOdometry()
        # self.setWheelsPWM(210, 190)
        # sleep(1.0)

    def doLongLine(self):
        if self.departureNode in [10, 5, 7, 1]: # if outer loop, just pass the intersection and follow the white lines
            self.avstate = AVState.doingShortStraightLine
            self.setWheelsPWM(220, 200)
            sleep(1.0)

            self.avstate = AVState.usingVision
            return
        # self.K = 3.2
        # self.B = 2.7
        # self.avstate = AVState.doingLongStraightLine
        # self.resetOdometry()
        if self.departureNode in [9]:
            self.setWheelsPWM(self.rightVeltoPWM(15), self.leftVeltoPWM(14))
            # self.setWheelsPWM(210, 190)
            sleep(4.4)
            self.avstate = AVState.usingVision
            return
        self.setWheelsPWM(self.rightVeltoPWM(14.5), self.leftVeltoPWM(14))
        # self.setWheelsPWM(210, 190)
        sleep(5)
        self.avstate = AVState.usingVision

    def doIntersection(self):
        if self.nextTurn == graph.Turn.right:
            self.doRightTurn(self.chooseRightTurn())
        elif self.nextTurn == graph.Turn.left:
            self.doLeftTurn()
        elif self.nextTurn == graph.Turn.straight:
            self.doLongLine()
        self.setWheelsPWM(self.rightVeltoPWM(self.vel), self.leftVeltoPWM(self.vel))

    def start(self):
        self.lastUpdateTime = time.time()
        # self.nextTurn, self.vel = next(self.directions)
        # self.avstate = AVState.usingVision

        # self.doIntersection()
        self.avstate = AVState.usingVision
        self.setWheelsPWM(150, 145)

        try:
            while (True):
                self.update()
        except KeyboardInterrupt:
            print("CTRL+C pressed")
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
        self.traveledDistance += deltaX
        self.traveledRightTick += deltaSR
        self.traveledLeftTick += deltaSL
        deltaTheta = math.atan2((deltaSR - deltaSL) / 2, self.wheelBase / 2)

        deltaXR = deltaX * math.cos(self.egoPos.theta + deltaTheta)
        deltaYR = deltaX * math.sin(self.egoPos.theta + deltaTheta)
        #
        # print("==============================")
        # print ("TIME={}".format(self.totalTimePassed))
        # print ("DaltaTIME={}".format(deltaTime))
        self.egoPos += Position(deltaXR, deltaYR, deltaTheta)
        # #
        self.deriveYokePos()
        # # print("yoke after: {}".format(self.yokePos))
        #
        self.currentLeftTick = lTick
        self.currentRightTick = rTick

        # ======= ODOMETRY =======
        if self.avstate in [AVState.doingRightTurn, AVState.doingLeftTurn, AVState.doingShortStraightLine, AVState.doingLongStraightLine]:
            # print ("DeltaSL: {}\tDeltaSR: {}".format(deltaSL, deltaSR))
            funcmap = {
                AVState.doingLongStraightLine: self.getRef,
                AVState.doingLeftTurn: self.getCircleRef
            }
            refFunction = funcmap[self.avstate]

            xDot = deltaXR / deltaTime
            yDot = deltaYR / deltaTime
            thetaDot = deltaTheta / deltaTime
            yokeDotPos = Position(xDot + self.distYoke * -math.sin(self.egoPos.theta) * thetaDot,
                                yDot + self.distYoke * math.cos(self.egoPos.theta) * thetaDot,
                                thetaDot)
            
            refPos = refFunction(self.totalTimePassed,14)
            # refPos = self.getCircleRef(self.totalTimePassed,self.vel)
            # print ("REF={}".format(refPos))
            
            errorP = self.yokePos - refPos
            errorD = Position(yokeDotPos.x - ((refPos.x - self.pastRef.x)/deltaTime),
                            yokeDotPos.y - ((refPos.y - self.pastRef.y)/deltaTime),
                            0)
            
            # print("ErrorP={}".format(errorP))
            # print("errorD={}".format(errorD))
            self.pastRef = refPos

            F_PD = (-1 * (self.K * errorP) )+ (-1 * (self.B * errorD))
            
            # print ("F_PD= {}".format(F_PD))
            
            # errorAngleFromBase = math.atan2(errorP.y, errorP.x)
            # eTheta =  errorAngleFromBase - self.egoPos.theta
            eTheta =  self.egoPos.theta
            
            # print("theta= {}, error theta= {}, projection angle= {}".format(self.egoPos.theta, errorAngleFromBase, eTheta))
            
            x_hat = math.cos(eTheta)
            y_hat = math.sin(eTheta)
            
            # print("xhat={}, yhat={}".format(x_hat,y_hat))
            
            F_trans = F_PD.x * x_hat + F_PD.y * y_hat
            F_rot =  F_PD.x * (y_hat) - F_PD.y * ( x_hat)

            print ("F_trans= {}\tF_rot= {}\tx= {}\ty= {}\tt= {}\txref= {}".format(F_trans, F_rot, self.yokePos.x, self.yokePos.y, self.egoPos.theta, refPos.x))
            # print ("F_rot= {}".format(F_rot))
            # print("x: {}".format(self.egoPos.x))

            rot_w = 0.8
            self.currentRightPWM += F_trans - (F_rot * rot_w)
            self.currentLeftPWM += F_trans + (F_rot * rot_w)

            # terminating condition
            if self.avstate == AVState.doingLongStraightLine:
                if self.egoPos.x > 65:
                    self.avstate = AVState.usingVision
                    return
            elif self.avstate == AVState.doingLeftTurn:
                if self.egoPos.theta > 1.215 and self.egoPos.x > 35:
                    self.avstate = AVState.usingVision
                    return
                
      
            self.setWheelsPWM(self.currentRightPWM, self.currentLeftPWM)
            return

        ##### END ODOMETRY ######

        if self.distance != 0 and self.distance < 35:
            self.__writeToSerial(500)
            return

        if self.avstate == AVState.usingVision or self.avstate == AVState.waitForGreenLight:
            ################### VISION ##############################
            redline = 0
            visionError = self.vision.error
            # if abs(visionError) < 20 and abs(visionError) > 10:
            #     if visionError < 0:
            #         visionError += -5
            #     else:
            #         visionError += 5
            if visionError != None and (visionError != 10000):
                visionErrorROC = (visionError - self.visionLastError) / deltaTime
                F_PD_vision = (-1 * (self.visionK * visionError)) + (-1 * (self.visionB * visionErrorROC))
                # print("F_PD_vision= {}, visionROC= {}, error= {}".format(F_PD_vision,visionErrorROC, visionError))

                self.visionLastError = visionError

                # cumulative error correction (disabled for the moment)
                self.currentRightPWM += -F_PD_vision
                self.currentLeftPWM += F_PD_vision

                # absolute error correction (not general, but holds speed better)
                # self.currentRightPWM = 145 + F_PD_vision
                # self.currentLeftPWM = 134 - F_PD_vision
                

                redline = self.vision.redline
                greenline = self.vision.greenline
                # print ("RedLine ======= {} {:.3f}".format(redline, time.time()))
                # print ("GreenLine ======= {}".format(greenline))

            ######################CHECK FOR SPEED#####################

            speed = deltaX / deltaTime
            # print ('deltaX={} Speed={}'.format(deltaX,speed))
            if speed < self.vel - 2:
                self.currentLeftPWM += 2
                self.currentRightPWM += 2

            if speed > self.vel + 10:
                self.currentLeftPWM -= 1
                self.currentRightPWM -= 1

            if self.avstate == AVState.usingVision:
                self.setWheelsPWM(self.currentRightPWM, self.currentLeftPWM)

            if redline > 350 and self.avstate == AVState.usingVision:
                self.__writeToSerial(500)
                self.avstate = AVState.waitForGreenLight
                try:
                    self.nextTurn, self.vel = next(self.directions)
                    self.departureNode = self.sequence.pop(0)
                    self.arrivalNode = self.sequence[0]
                    print("Departure:{}  Arrival:{}".format(self.departureNode, self.arrivalNode))
                except StopIteration:
                    print("We are Done")
                    self.stop()
                    exit()
                sleep(0.1)

            if self.avstate == AVState.waitForGreenLight:
                # TODO: add detecting green
                # greenline = 10
                # sleep(3)
                # print ("GreenLine ======= {}".format(greenline))
                # print("waiting for green")
                if greenline > 30:
                    sleep(0.15)
                    # print("frame={}, error={}, gtCenter={}, rightLane={}, leftLane={}".format(self.frameNumber, visionError, self.vision.gtCenter,
                    #                                                                       self.vision.rightLane, self.vision.leftLane))
                    print("see green light >>>>>>>>>>> {}".format(greenline))
                    if self.vision.markedFrame is not None:
                        Image.fromarray(self.vision.markedFrame).save("../sampleimg/test/{}.jpg".format(self.frameNumber))
                        Image.fromarray(self.vision.greenImage).save("../sampleimg/test/greenMasked-{}.jpg".format(self.frameNumber))

                        self.frameNumber += 1
                    self.doIntersection()
                        # Print the element

                # else:
                #     print("no green")
        # print("self.vision.error:{}  self.ego.theta:{}".format(self.vision.error, self.egoPos.theta))

    def stop(self):
        print("Stop")
        self.__writeToSerial(500)
        self.vision.done = True
        self.camera.stop_recording()
        # self.vision.stopRecording()
        sleep(1)

    def rightPWMtoVel(self, PWM):
        slope = 0.17250446
        intercept = -14.93387235
        return PWM * slope + intercept

    def rightVeltoPWM(self, vel):
        slope = 5.75863643
        intercept = 87.713437
        return int(vel * slope + intercept)

    def leftPWMtoVel(self, PWM):
        slope = 0.17139429
        intercept = -13.02985714
        return PWM * slope + intercept

    def leftVeltoPWM(self, vel):
        slope = 5.82088733
        intercept = 76.42862371
        return int(vel * slope + intercept)


def main():
    parser = argparse.ArgumentParser(description='AVDocky')
    # parser.add_argument("-p", help='Port of the serial' ,required=True)
    parser.add_argument("-p", help='Port of the serial', default="/dev/ttyACM0")
    parser.add_argument("-R", help='Rate of the serial', default=115200)
    parser.add_argument("-r", type=int, help='Right wheel PWM', default=400)
    parser.add_argument("-l", type=int, help='Left wheel PWM', default=400)
    parser.add_argument("-speed", type=float, help='Speed of going straight cm/s', default=10)

    parser.add_argument("-x", type=float, help="X position", default=3000)
    parser.add_argument("-y", type=float, help="Y position", default=0)
    parser.add_argument("-th", type=float, help="Theta", default=0)
    parser.add_argument("-seq", type=str, help="Sequence of the road", default="2,6,3,1,10,7,6,4,9,1,3")
    args = parser.parse_args()

    nums = args.seq.split(",")
    sequence = [int(n) for n in nums]
    directions, sequence = graph.get_duckietown_tour(sequence)

    # for i in range(len(sequence)):
    #     sequence[i] = sequence[i] + 1
    # print(sequence)
    # print_dr = copy.copy(directions)

    av = AV(args.p, args.R, directions, sequence)
    # print(list(directions))
    # print(av.directions)
    av.vel = args.speed
    atexit.register(av.stop)
    # print(av.sequence)
    # av.goalPos = Position(args.x,args.y,args.th)
    av.start()
    # sleep(1)
    # av.stop()
    # exit()
    # av.setWheelsPWM(av.rightVeltoPWM(args.speed), av.leftVeltoPWM(args.speed))

    sleep(2)


if __name__ == '__main__':
    main()
