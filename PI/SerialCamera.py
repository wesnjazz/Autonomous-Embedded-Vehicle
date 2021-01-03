# this was built to test image processing with no multithreading
# I learned that or HSV conversion was really slow
# and that solved the multithreading issues, so this file is
# basically useless now

import io
import time
from PIL import Image
import picamera
import colorsys
import numpy as np

class SplitFrames(object):
    def __init__(self):
        self.frame_num = 0
        self.stream = io.BytesIO()
        self.last_process_time = time.time()

        camW, camH = 640, 480

        ROITop=230
        ROIHeight= 40
        ROILeft=0
        ROIRight=0
        ROITop=230
        ROIHeight= 40

        self.ROITop = ROITop
        self.ROIHeight = ROIHeight
        self.ROILeft = ROILeft
        self.ROIRight = camW - ROIRight

        self.rightLane, self.leftLane = None, None
        self.error = 0
 
        self.gtCenter = None
        self.gtCenter = None
        self.gtFromLeftLane = None
        self.gtFromRightLane = None


    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # process the old image
            if self.frame_num > 0:
                self.stream.seek(0)
                img = Image.open(self.stream)

                self.processImage(img)
                
                curr_time = time.time()
                print('time between frames: {}'.format(curr_time - self.last_process_time))
                self.last_process_time = curr_time
                
                # print(img)


            # Start of new frame; close the old one (if any) and
            # open a new output
            # Reset the stream and event
            self.stream.seek(0)
            self.stream.truncate()
            # self.event.clear()
            self.frame_num += 1

            
        # self.output.write(buf)
        self.stream.write(buf)

    def RGBtoHSV(self,rgb):
        return rgb.convert('HSV')
        # input_shape = rgb.shape
        # rgb = rgb.reshape(-1, 3)
        # r, g, b = rgb[:, 0], rgb[:, 1], rgb[:, 2]

        # maxc = np.maximum(np.maximum(r, g), b)
        # minc = np.minimum(np.minimum(r, g), b)
        # v = maxc

        # deltac = maxc - minc
        # s = deltac / maxc
        # deltac[deltac == 0] = 1  # to not divide by zero (those results in any way would be overridden in next lines)
        # rc = (maxc - r) / deltac
        # gc = (maxc - g) / deltac
        # bc = (maxc - b) / deltac

        # h = 4.0 + gc - rc
        # h[g == maxc] = 2.0 + rc[g == maxc] - bc[g == maxc]
        # h[r == maxc] = bc[r == maxc] - gc[r == maxc]
        # h[minc == maxc] = 0.0

        # h = (h / 6.0) % 1.0
        # res = np.dstack([h, s, v])
        # return res.reshape(input_shape)


    def processImage(self,image):
        startTime = time.time()

        # print('==== frame')

        # sleep(0.1)

        # self.error = 0

        # return

        # the above code proves that the image processing takes too long - it takes longer than one control loop update

        # im.crop((left, top, right, bottom))
        cropped = image.crop((self.ROILeft, self.ROITop, self.ROIRight, self.ROITop+self.ROIHeight))
        hsvImage = self.RGBtoHSV(cropped)
        numarray = np.array(hsvImage)

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
        if left_lane_x > right_lane_x:
            right_lane_x = 0

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

        if center_of_lane > markedImageArray.shape[1]:
            center_of_lane = markedImageArray.shape[1] -1



with picamera.PiCamera(resolution=(640, 480), framerate=30) as camera:
    camera.start_preview()
    # Give the camera some warm-up time
    time.sleep(2)
    output = SplitFrames()
    start = time.time()
    camera.start_recording(output, format='mjpeg')
    camera.wait_recording(5)
    camera.stop_recording()
    finish = time.time()
print('Captured %d frames at %.2ffps' % (
    output.frame_num,
    output.frame_num / (finish - start)))