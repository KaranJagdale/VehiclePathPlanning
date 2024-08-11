import numpy as np
import matplotlib.pyplot as plt
import csv

xArr, yArr = [], []
#reading the path
with open('res.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        loc = list(map(float, row[0].split(',')))
        xArr.append(loc[0])
        yArr.append(loc[1])
            
csvfile.close()

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

print(envBoundary, envObjects)
plt.plot(xArr, yArr)
plt.xticks(np.arange(envBoundary[0], envBoundary[2], 1))
plt.yticks(np.arange(envBoundary[1], envBoundary[3], 1))
plt.grid()
ax = plt.gca()
ax.set_xlim(envBoundary[0], envBoundary[2])
ax.set_ylim(envBoundary[1], envBoundary[3])

for i in range(len(envObjects)):
    xFill = [envObjects[i][0], envObjects[i][2]]
    y1, y2 = envObjects[i][1], envObjects[i][3]
    plt.fill_between(xFill, y1, y2)
plt.show()