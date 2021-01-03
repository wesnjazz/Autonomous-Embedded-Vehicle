# import picamera
from PIL import Image
import numpy as np
import colorsys 

im = Image.open("teamimage1.jpg")
# hsv = im.convert('HSI')
# im.show()
# hsv.show()


def HSVColor(img):
    if isinstance(img,Image.Image):
        r,g,b = img.split()
        Hdat = []
        Sdat = []
        Vdat = [] 
        for rd,gn,bl in zip(r.getdata(),g.getdata(),b.getdata()) :
            h,s,v = colorsys.rgb_to_hsv(rd/255.,gn/255.,bl/255.)
            Hdat.append(int(h*255.))
            Sdat.append(int(s*255.))
            Vdat.append(int(v*255.))
        r.putdata(Hdat)
        g.putdata(Sdat)
        b.putdata(Vdat)
        return Image.merge('RGB',(r,g,b)), r
    else:
        return None


w, h = im.size
boxh = 20
# cropped = im.crop((400,h/2-boxh+5,450,h/2+boxh-30))
# im.crop((left, top, right, bottom))

cropped = im.crop((415,h/2-boxh-5,445,h/2+boxh-15))
hsv, r = HSVColor(cropped)
hsvarray = np.asarray(r)
print(hsvarray)
# print(hsvarray[1,1,:])
# for i in range(hsvarray.shape[0]):
# 	for j in range(hsvarray.shape[1]):
# 		print(hsvarray[i,j,:])


cropped.show()
# hsv.show()
# r.show()
# cropped.save("foo.jpg")
# yRGB = np.asarray(cropped)
# print(yRGB)
# print(yRGB.shape)
# print(yRGB[20,20])

# print(cropped.size)
# croppedHSV = HSVColor(cropped)
# print(type(croppedHSV))
# croppedHSV = croppedHSV.crop((140,h/2-boxh-10,170,h/2+boxh-10))
# yHSV = np.asarray(croppedHSV)
# print(yHSV[20,20])
# hsv.show()
