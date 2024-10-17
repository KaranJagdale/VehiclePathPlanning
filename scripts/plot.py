import numpy as np
import matplotlib.pyplot as plt
import csv
import matplotlib.animation as animation
from functools import partial

xArr, yArr = [], []
#reading the path
with open('res.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        loc = list(map(float, row[0].split(',')))
        xArr.append(loc[0])
        yArr.append(loc[1])
            
csvfile.close()

xArrSmooth, yArrSmooth = [], []
#reading the path
with open('smoothPath.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        loc = list(map(float, row[0].split(',')))
        xArrSmooth.append(loc[1])
        yArrSmooth.append(loc[2])
            
csvfile.close()

xArrSim, yArrSim = [], []
#reading the path
with open('simulationRes.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        loc = list(map(float, row[0].split(',')))
        xArrSim.append(loc[1])
        yArrSim.append(loc[2])
            
csvfile.close()

#reversing xArr and yArr
xArr = xArr[::-1]
yArr = yArr[::-1]

# reading the environment
envBoundary = []
envObjects = []
with open('env.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    itr = 0
    for row in spamreader:
        if itr == 0:
            envBoundary = list(map(float, row[0].split(',')))
        else:
            envObjects.append(list(map(float, row[0].split(','))))
        itr += 1


def update(frame, x_arr, y_arr, line): #function for creating animation
    x = x_arr[0: frame + 1]
    y = y_arr[0: frame + 1]

    line.set_xdata(x)
    line.set_ydata(y)
    return line


fig, ax = plt.subplots()
plt.xticks(np.arange(envBoundary[0], envBoundary[2], 1))
plt.yticks(np.arange(envBoundary[1], envBoundary[3], 1))
ax.grid()
ax.set_xlim(envBoundary[0], envBoundary[2])
ax.set_ylim(envBoundary[1], envBoundary[3])

for i in range(len(envObjects)):
    xFill = [envObjects[i][0], envObjects[i][2]]
    y1, y2 = envObjects[i][1], envObjects[i][3]
    ax.fill_between(xFill, y1, y2)

#plotting start and target nodes
ax.plot(xArr[0] , yArr[0], 'bo')
ax.plot(xArr[len(xArr) - 1], yArr[len(xArr) - 1], 'rx')

line = ax.plot(0,0)[0]

simDelay = 500 #time between consecutive frames

ani = animation.FuncAnimation(fig=fig, func=partial(update, x_arr = xArr, y_arr = yArr , line = line), frames=range(len(xArr) - 1), interval = simDelay)
    #writervideo = animation.FFMpegWriter(fps=60) 
ani.save('res.gif')#, writer=writervideo) 


# creating a resultant plot
fig1, ax1 = plt.subplots()
plt.xticks(np.arange(envBoundary[0], envBoundary[2], 1))
plt.yticks(np.arange(envBoundary[1], envBoundary[3], 1))
ax1.grid()
ax1.set_xlim(envBoundary[0], envBoundary[2])
ax1.set_ylim(envBoundary[1], envBoundary[3])

for i in range(len(envObjects)):
    xFill = [envObjects[i][0], envObjects[i][2]]
    y1, y2 = envObjects[i][1], envObjects[i][3]
    ax1.fill_between(xFill, y1, y2)
ax1.plot(xArr[0] , yArr[0], 'bo')
ax1.plot(xArr[len(xArr) - 1], yArr[len(xArr) - 1], 'rx')
ax1.plot(xArr, yArr)
ax1.plot(xArrSmooth, yArrSmooth)
ax1.plot(xArrSim, yArrSim)


plt.show()
plt.close()