# used to investigate hsv thresholds on images

from PIL import Image
import numpy as np


img = Image.open('../sampleimg/test/200.jpg')

arr = np.array(img)

hsv = np.array(img.convert('HSV'))

x, y = 295, 206
print(arr[y, x])
print(hsv[y, x])

# print(hsv[y:y+10, x:x+10])

# for x in range(hsv.shape[1]):
#     for y in range(hsv.shape[0]):
#         if hsv[y, x, 1] > 0:
#             print(x, y, hsv[y, x])

# print(hsv)