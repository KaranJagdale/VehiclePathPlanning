import numpy
import matplotlib.pyplot as plt
import csv

x_arr, y_arr = [], []

with open('res.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        for i in range(len(row[0])):
            if row[0][i] == ',':
                x_arr.append(float(row[0][0:i]))
                y_arr.append(float(row[0][i+1:len(row[0])]))
        # x_arr.append(row[0])
        # y_arr.append(row[1])

plt.plot(x_arr, y_arr)
plt.show()