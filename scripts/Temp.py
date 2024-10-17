import numpy as np
import matplotlib.pyplot as plt
import csv
import matplotlib.animation as animation
from functools import partial
from scipy.interpolate import CubicSpline
import math

timeArr, xArr, yArr, thetaArr = [], [], [], []
#reading the path
with open('smoothPath.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        loc = list(map(float, row[0].split(',')))
        timeArr.append(loc[0])
        xArr.append(loc[1])
        yArr.append(loc[2])
        thetaArr.append(loc[3])
            
csvfile.close()

thetaPlanner = []
timePlanner = []
with open('res.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    i = 0
    for row in spamreader:
        loc = list(map(float, row[0].split(',')))
        thetaPlanner.append(loc[2])
        timePlanner.append(i)
        i += 1/10

csvfile.close()

thetaFromDiff = []

for i in range(len(xArr) - 1):
    thetaFromDiff.append(math.atan2((yArr[i+1] - yArr[i]), (xArr[i+1] - xArr[i])))

plt.plot(timePlanner, thetaPlanner[::-1])
plt.plot(timeArr, thetaArr)
plt.plot(timeArr[:len(timeArr) - 1], thetaFromDiff)


print(math.atan2(1,1))

plt.show()


