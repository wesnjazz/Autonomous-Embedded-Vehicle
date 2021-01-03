# from vision import CameraHandler
from vision2 import ProcessOutput
from PIL import Image
from os import listdir
from os.path import isfile, join
import time
from picamera import PiCamera
from PIL import Image


mypath = "../sampleimg/20secImgs/"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]

# ch = CameraHandler(300, 30)
# vision = CameraHandler(ROITop=200,ROIHeight= 30,ROIRight=50)
# vision.startRecording()
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32

camera.start_preview()
time.sleep(2)
vision = ProcessOutput(200,30)
camera.start_recording(vision, format='mjpeg')
print ("============ after  start ================")
frameNumber = 0
now = time.time()
try:
    while True:
        # if vision.frameReady:
        #     markedFrame = vision.readFrame()

        if vision.markedFrame != None:
            # Image.fromarray(vision.markedFrame).save("../sampleimg/test/{}.jpg".format(frameNumber))
            print("dtime={}, frame={}, gtCenter={}, rightLane={}, leftLane={}".format(time.time() - now, frameNumber,
                                                                                      vision.gtCenter,
                                                                                      vision.rightLane,
                                                                                      vision.leftLane))
            now = time.time()

            frameNumber +=1
        pass
except KeyboardInterrupt:
    print ("CTRL+C pressed")
    vision.done = True
    camera.stop_recording()
    print('done')

