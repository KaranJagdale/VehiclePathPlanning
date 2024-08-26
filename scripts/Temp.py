import numpy as np
import matplotlib.pyplot as plt
import csv
import matplotlib.animation as animation
from functools import partial
from scipy.interpolate import CubicSpline

xArr, yArr = [], []
#reading the path
with open('res.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        loc = list(map(float, row[0].split(',')))
        xArr.append(loc[0])
        yArr.append(loc[1])
            
csvfile.close()

#reversing xArr and yArr
xArr = xArr[::-1]
yArr = yArr[::-1]

timeTick = np.linspace(0, len(xArr) - 1, len(xArr))

cs_x = CubicSpline(timeTick, xArr)
cs_y = CubicSpline(timeTick, yArr)

timeTickFine = np.arange(0, len(xArr) - 1, 0.1)

plt.plot(xArr, yArr)
plt.plot(cs_x(timeTickFine), cs_y(timeTickFine))
plt.show()


