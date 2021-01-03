from picamera.array import PiRGBArray
from picamera import PiCamera
# from threading import Thread
from multiprocessing import Process
from time import sleep
import numpy as np
from PIL import Image
import time
import colorsys


class CameraHandler():
    def __init__(self, ROITop, ROIHeight,ROILeft=0,ROIRight=0,camW=640,camH=480,framerate=32):
        self.ROITop = ROITop
        self.ROIHeight = ROIHeight
        self.ROILeft = ROILeft
        self.ROIRight = camW - ROIRight
        self.camW = camW
        self.camH = camH

        self.gtCenter = None
        self.gtFromRightLane = None
        self.gtFromLeftLane = None
        self.rightLane = None
        self.leftLane = None

        self.frame = None
        self.markedFrame = None
        self.error = None
        self.stopCapturing = False

        resolution = (camW, camH)
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
                                                     format="rgb", use_video_port=True)

        self.frameReady = False

        self.lastFrameTime = time.time()

    def stopRecording(self):
        self.stopCapturing = True

    def startRecording(self):
        # start the thread to read frames from the video stream
        # Thread(target=self.update, args=()).start()
        p = Process(target=self.update, args=())
        p.start()
        return self

    def readFrame(self):
        self.frameReady = False
        return self.markedFrame

    def update(self):
        print('update started')

        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame

            print('=== frame start')
            lastFrameTime = time.time()
            self.frame = f.array
            self.processImage(Image.fromarray(self.frame))
            self.frameReady = True
            self.rawCapture.truncate(0)
            

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopCapturing:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

            print('=== frame end {}'.format(time.time() - lastFrameTime))

    def RGBtoHSV(self,img):
        if isinstance(img, Image.Image):
            r, g, b = img.split()
            Hdat = []
            Sdat = []
            Vdat = []
            for rd, gn, bl in zip(r.getdata(), g.getdata(), b.getdata()):
                h, s, v = colorsys.rgb_to_hsv(rd / 255., gn / 255., bl / 255.)
                Hdat.append(int(h * 255.))
                Sdat.append(int(s * 255.))
                Vdat.append(int(v * 255.))
            r.putdata(Hdat)
            g.putdata(Sdat)
            b.putdata(Vdat)
            return Image.merge('RGB', (r, g, b)), r
        else:
            return None

    def processImage(self,image):
        startTime = time.time()

        # print('==== frame')

        # sleep(0.1)

        # self.error = 0

        # return

        # the above code proves that the image processing takes too long - it takes longer than one control loop update

        # im.crop((left, top, right, bottom))
        cropped = image.crop((self.ROILeft, self.ROITop, self.ROIRight, self.ROITop+self.ROIHeight))
        # hsvImage, hImage = self.RGBtoHSV(cropped)
        numarray = np.array(cropped)

        print('converted to hsv, {} sec'.format(time.time() - startTime))
        startTime = time.time()


        # find column with the most yellow pixels
        # if the hue is between 28 and 50, and the stauration is high
        mask_left = (28 < numarray[:,:,0]) & (numarray[:,:,0] < 50) & (numarray[:,:,1] > 128)
        yellows = mask_left.sum(axis=0)
        # print(yellows)
        left_lane_x = yellows.argmax()

        # find column with the most white pixels
        # if the saturation is low and the value is high
        mask_right = (numarray[:,:,1] < 20) & (220 < numarray[:,:,2])
        whites = mask_right.sum(axis=0)
        right_lane_x = whites.argmax()

        print('found lanes, {} sec'.format(time.time() - startTime))

        self.rightLane = right_lane_x
        self.leftLane = left_lane_x

        center_of_lane = 0
        # calculate error
        if left_lane_x == 0 and right_lane_x == 0:
            # we're on the wrong side of the road, pull right
            self.error = 10000
        elif left_lane_x != 0 and right_lane_x != 0:
            # find center vs. center of camera
            center_of_lane = (right_lane_x + left_lane_x) / 2
            if self.gtCenter == None:
                self.gtCenter = center_of_lane
                self.gtFromLeftLane = center_of_lane - left_lane_x
                self.gtFromRightLane = center_of_lane - right_lane_x
            self.error = self.gtCenter - center_of_lane
        elif left_lane_x == 0 and right_lane_x != 0:
            center_of_lane = (right_lane_x + self.gtFromRightLane)
            self.error = self.gtCenter - center_of_lane
        elif left_lane_x != 0 and right_lane_x == 0:
            center_of_lane = (left_lane_x + self.gtFromLeftLane)
            self.error = self.gtCenter - center_of_lane

        # print('='*40 + ' vision error: {}'.format(self.error))

        markedImageArray = np.array(image)

        #left line
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.ROILeft, 0] = 66
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.ROILeft, 1] = 245
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.ROILeft, 2] = 96

        #right line
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.ROIRight-1, 0] = 66
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.ROIRight-1, 1] = 245
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.ROIRight-1, 2] = 96

        #top line
        markedImageArray[self.ROITop, self.ROILeft:self.ROIRight-1, 0] = 66
        markedImageArray[self.ROITop, self.ROILeft:self.ROIRight-1, 1] = 245
        markedImageArray[self.ROITop, self.ROILeft:self.ROIRight-1, 2] = 96

        #bottom line
        markedImageArray[(self.ROITop + self.ROIHeight), self.ROILeft:self.ROIRight-1, 0] = 66
        markedImageArray[(self.ROITop + self.ROIHeight), self.ROILeft:self.ROIRight-1, 1] = 245
        markedImageArray[(self.ROITop + self.ROIHeight), self.ROILeft:self.ROIRight-1, 2] = 96

        # # show the lane boundary
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), left_lane_x, 0] = 255
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), left_lane_x, 1] = 0
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), left_lane_x, 2] = 0

        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), right_lane_x, 0] = 255
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), right_lane_x, 1] = 0
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), right_lane_x, 2] = 0

        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.gtCenter, 0] = 3
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.gtCenter, 1] = 248
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.gtCenter, 2] = 252

        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), center_of_lane, 0] = 219
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), center_of_lane, 1] = 3
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), center_of_lane, 2] = 252

        self.markedFrame = markedImageArray
        # final_img = Image.fromarray(croppedArray)
        # final_img.save(savePath)

        # check that the lane identifications are reliable
        # TODO indicate that odometery must be used in this situation
        # if left_lane_x < 3:
        #     print('weak left lane signal: {}'.format(mask_left[left_lane_x]))
        # if right_lane_x < 3:
        #     print('weak right lane signal: {}'.format(mask_right[right_lane_x]))



        # TODO use error to drive wheels

