import io
import time
import threading
import picamera
from PIL import Image
import numpy as np
import colorsys
from time import sleep
import graph

# takes a PIL image, returns a PIL image
def RGBtoHSV(rgb):
    return rgb.convert('HSV')

class ImageProcessor(threading.Thread):
    def __init__(self, owner, AV, ROITop, ROIHeight, ROILeft, ROIRight, resolution=(640, 480)):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.owner = owner
        self.start()
        self.AV = AV

        camW, camH = resolution

        self.ROITop = ROITop
        self.ROIHeight = ROIHeight
        self.ROILeft = ROILeft
        self.ROIRight = camW - ROIRight


    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    self.stream.seek(0)
                    # print('got an image')
                    self.owner.frame = self.stream
                    # Read the image and do some processing on it
                    img = Image.open(self.stream)

                    self.processImage(img)
                    #...
                    #...
                    # Set done to True if you want the script to terminate
                    # at some point
                    self.owner.done=True
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        self.owner.pool.append(self)

    def processImage(self,image):
        startTime = time.time()
        # print("\t\t\t\t\t\t Turn:{}  {:.3f}".format(self.AV.nextTurn, time.time()))
        # im.crop((left, top, right, bottom))
        cropped = image.crop((self.ROILeft, self.ROITop, self.ROIRight, self.ROITop+self.ROIHeight))
        hsv = RGBtoHSV(cropped)
        numarray = np.array(hsv)

        # print('converted to hsv, {} sec'.format(time.time() - startTime))
        startTime = time.time()


        # find column with the most yellow pixels
        # if the hue is between 28 and 50, and the stauration is high
        mask_left = (28 < numarray[:,:,0]) & (numarray[:,:,0] < 50) & (numarray[:,:,1] > 128)
        yellows = mask_left.sum(axis=0)
        # print(yellows)
        left_lane_x = yellows.argmax()
        # if left_lane_x == 0:
        #     print("left_lane_x:{}".format(left_lane_x))
        #     print("value of yellows:{}".format(yellows[left_lane_x]))
        # else:
            # left_lane_x += 20
        # if left_lane_x == 0 and yellows[left_lane_x] > 20:
        #     left_lane_x = 5

        # find column with the most white pixels
        # if the saturation is low and the value is high
        mask_right = (numarray[:,:,1] < 20) & (220 < numarray[:,:,2])
        whites = mask_right.sum(axis=0)
        right_lane_x = whites.argmax()
        # if right_lane_x < (self.ROIRight - 11):
        #     right_lane_x += 10

        #find red box
        mask_red = ((numarray[:, :, 0] < 14) | (numarray[:, :, 0] > 240)) & (120 < numarray[:, :, 1]) & (150 < numarray[:, :, 2])
        red = mask_red.sum(axis=1)
        red_line_y = red.argmax()
        green_line_y = 0
        self.owner.greenline = 0
        boundry = 639
        if red_line_y > 60:
            self.owner.redline = red[red_line_y]
            # find green light
            offset = 100
            greencropped = image.crop((self.ROILeft, self.ROITop-offset, self.ROIRight, self.ROITop + self.ROIHeight+offset))
            greenImage = np.array(greencropped)
            hsv = RGBtoHSV(greencropped)
            greenNumarray = np.array(hsv)

            mask_green = ((greenNumarray[:, :, 0] > 43) & (greenNumarray[:, :, 0] < 118) & (greenNumarray[:, :, 1] >= 20 ) & (200 < greenNumarray[:, :, 2]))
            green = mask_green.sum()
            # greenImage[mask_green,:] = 1 
            greenImage[~mask_green,:] = 0
            # green_line_y = green.argmax()
            # self.owner.greenline = green[green_line_y]
            self.owner.greenline = green
            self.owner.greenImage = greenImage

        else:
            self.owner.redline = 0
            self.owner.greenline = 0


        # print('found lanes, {} sec'.format(time.time() - startTime))

        self.owner.rightLane = right_lane_x
        self.owner.leftLane = left_lane_x

        center_of_lane = 0
        # calculate error
        if left_lane_x > right_lane_x:
            right_lane_x = 0

        if left_lane_x == 0 and right_lane_x == 0 and self.owner.redline == 0:
            # we're on the wrong side of the road, pull right
            # self.owner.error = 10000
            self.owner.error = 0
        elif left_lane_x != 0 and right_lane_x != 0:
            # find center vs. center of camera
            center_of_lane = (right_lane_x + left_lane_x) / 2 # + 50 # vehicle was a little bit left sided from center
            if self.owner.gtCenter == None:
                self.owner.gtCenter = center_of_lane
                self.owner.gtFromLeftLane = center_of_lane - left_lane_x
                self.owner.gtFromRightLane = center_of_lane - right_lane_x  # outer loop
            # print("red_line_y:{}".format(red_line_y))

            self.owner.error = self.owner.gtCenter - center_of_lane
            # if red_line_y > 20:
            #     self.owner.error += 50
            # print('error:{}'.format(self.owner.error))
        elif left_lane_x == 0 and right_lane_x != 0:
            center_of_lane = (right_lane_x + self.owner.gtFromRightLane) #+ 55
            self.owner.error = self.owner.gtCenter - center_of_lane
        elif left_lane_x != 0 and right_lane_x == 0:
            center_of_lane = (left_lane_x + self.owner.gtFromLeftLane) # + 50
            self.owner.error = self.owner.gtCenter - center_of_lane
            # print('error:{}'.format(self.owner.error))

        # print('='*40 + ' vision error: {}'.format(self.error))

        markedImageArray = np.array(image)

        if center_of_lane > markedImageArray.shape[1]:
            center_of_lane = markedImageArray.shape[1] -1

        # mark red line
        markedImageArray[self.ROITop + red_line_y, :, 0] = 66
        markedImageArray[self.ROITop + red_line_y, :, 1] = 69
        markedImageArray[self.ROITop + red_line_y, :, 2] = 245

        #mark green line
        # markedImageArray[self.ROITop + green_line_y, :, 0] = 255
        # markedImageArray[self.ROITop + green_line_y, :, 1] = 166
        # markedImageArray[self.ROITop + green_line_y, :, 2] = 0

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

        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.owner.gtCenter, 0] = 3
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.owner.gtCenter, 1] = 248
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), self.owner.gtCenter, 2] = 252

        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), center_of_lane-1, 0] = 219
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), center_of_lane-1, 1] = 3
        markedImageArray[self.ROITop:(self.ROITop + self.ROIHeight), center_of_lane-1, 2] = 252

        self.owner.markedFrame = markedImageArray
        # final_img = Image.fromarray(croppedArray)
        # final_img.save(savePath)

        # check that the lane identifications are reliable
        # TODO indicate that odometery must be used in this situation
        # if left_lane_x < 3:
        #     print('weak left lane signal: {}'.format(mask_left[left_lane_x]))
        # if right_lane_x < 3:
        #     print('weak right lane signal: {}'.format(mask_right[right_lane_x]))
    def adjustCenterLane(self, color, center_of_lane):
        colorStr = ""
        if color == 1:
            colorStr = 'YELLOW'
        elif color == 2:
            colorStr = 'WHITE'
        elif color == 3:
            colorStr = 'BOTH yellow and white'

        if self.AV.departureNode == 7 and self.AV.arrivalNode == 1 or \
                self.AV.departureNode == 10 and self.AV.arrivalNode == 2 or \
                self.AV.departureNode == 5 and self.AV.arrivalNode == 3 or \
                self.AV.departureNode == 1 and self.AV.arrivalNode == 4 or \
                self.AV.departureNode == 3 and self.AV.arrivalNode == 12:
            # print("{} to {}, {}, LEFT small {}".format(self.AV.departureNode, self.AV.arrivalNode, colorStr, time.time()))
            center_of_lane += 0
        elif self.AV.departureNode == 5 and self.AV.arrivalNode == 7 or \
                self.AV.departureNode == 4 and self.AV.arrivalNode == 7 or \
                self.AV.departureNode == 7 and self.AV.arrivalNode == 10 or \
                self.AV.departureNode == 8 and self.AV.arrivalNode == 10 or \
                self.AV.departureNode == 12 and self.AV.arrivalNode == 5 or \
                self.AV.departureNode == 10 and self.AV.arrivalNode == 5:
            # print("{} to {}, {}, LEFT big {}".format(self.AV.departureNode, self.AV.arrivalNode, colorStr, time.time()))
            center_of_lane += 70
        elif self.AV.departureNode == 9 and self.AV.arrivalNode == 1 or \
                self.AV.departureNode == 9 and self.AV.arrivalNode == 6 or \
                self.AV.departureNode == 11 and self.AV.arrivalNode == 9 or \
                self.AV.departureNode == 12 and self.AV.arrivalNode == 9 or \
                self.AV.departureNode == 6 and self.AV.arrivalNode == 11:
            # print("{} to {}, {}, RIGHT big {}".format(self.AV.departureNode, self.AV.arrivalNode, colorStr, time.time()))
            center_of_lane -= 25
        elif self.AV.departureNode == 8 and self.AV.arrivalNode == 6 or \
                self.AV.departureNode == 6 and self.AV.arrivalNode == 3 or \
                self.AV.departureNode == 3 and self.AV.arrivalNode == 8 or \
                self.AV.departureNode == 11 and self.AV.arrivalNode == 2 or \
                self.AV.departureNode == 2 and self.AV.arrivalNode == 4 or \
                self.AV.departureNode == 4 and self.AV.arrivalNode == 11:
            # print("{} to {}, {}, RIGHT small {}".format(self.AV.departureNode, self.AV.arrivalNode, colorStr, time.time()))
            center_of_lane -= -5

        return center_of_lane


class ProcessOutput(object):
    def __init__(self, AV, ROITop, ROIHeight,ROILeft=0,ROIRight=0,framerate=32, camW=640, camH=480):
        self.done = False
        # Construct a pool of 4 image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()
        self.pool = [ImageProcessor(self, AV, ROITop, ROIHeight, ROILeft, ROIRight) for i in range(1)]
        self.processor = None
        self.AV = AV

        self.camW = camW
        self.camH = camH

        self.gtCenter = 300
        self.gtFromRightLane = -245
        self.gtFromLeftLane = 245
        self.rightLane = None
        self.leftLane = None

        self.frame = None
        self.markedFrame = None
        self.error = None
        self.redline = None
        self.greenline = None
        self.stopCapturing = False
        self.greenImage = None

        resolution = (camW, camH)
        # self.camera = PiCamera()
        # self.camera.resolution = resolution
        # self.camera.framerate = framerate
        # self.rawCapture = PiRGBArray(self.camera, size=resolution)
        # self.stream = self.camera.capture_continuous(self.rawCapture,
        #                                              format="rgb", use_video_port=True)


    # def startRecording(self):
    #     self.camera.start_preview()
    #     time.sleep(2)
    #     camera.start_recording(output, format='mjpeg')

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.processor:
                self.processor.event.set()
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    # this frame; you may want to print a warning
                    # here to see whether you hit this case
                    self.processor = None
        if self.processor:
            self.processor.stream.write(buf)

    def flush(self):
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        # Now, empty the pool, joining each thread as we go
        while True:
            with self.lock:
                try:
                    proc = self.pool.pop()
                except IndexError:
                    pass # pool is empty
            try:
                proc.terminated = True
                proc.join()
            except NameError:
                print("proc not exists")
                print(len(self.pool), self.pool)
                self.stop()
                exit()

# if __name__ == '__main__':
#     with picamera.PiCamera(resolution=(640, 480)) as camera:
#         try:
#             camera.start_preview()
#             time.sleep(2)
#             output = ProcessOutput()
#             camera.start_recording(output, format='mjpeg')
#             while not output.done:
#                 camera.wait_recording(1)
#             camera.stop_recording()
#         except KeyboardInterrupt:
#             output.done = True
#             camera.stop_recording()
#             print('done')
