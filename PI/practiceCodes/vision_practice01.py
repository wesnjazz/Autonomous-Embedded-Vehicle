import picamera
import time
import numpy
from PIL import Image


while True:
    with picamera.PiCamera() as camera:
        camera.resolution = (1024,768)
        camera.start_preview()
        time.sleep(2)
        camera.capture('foo.jpg')

    im = Image.open(r'foo.jpg')
    w, h = im.size
    print("image size:({},{})".format(w,h))
    
    boxwidth = w
    boxheight = h/4 + 10
    imcrop = im.crop((0, h/4, boxwidth, boxheight))
    wc, hc = imcrop.size
    print("crop size:({},{})".format(wc,hc))
    
    x = 0
    y = 0
    a = imcrop.getpixel((x,y))
    print("pixel at ({},{}):{}".format(x,y,a))
    
    
    imnum = numpy.asarray(imcrop)
    print(imnum.shape)
    #print(imnum)
    time.sleep(1)
