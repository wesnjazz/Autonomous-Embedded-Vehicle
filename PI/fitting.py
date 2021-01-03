import numpy as np


x_l = [100,150,200,250,300,350,400]
x_r = [108,160,210,258,320,360,400]

y = [4.275,13.405,21.598,28.011,37.879,47.383,56.180]

import warnings

z = np.polyfit(x_r, y, 1)
# k = np.multiply(x_r, z[0]) + z[1]

# slope = 0.17139429
# intercept = -13.02985714

print(z)
# print(k)
