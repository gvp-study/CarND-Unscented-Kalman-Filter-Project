import matplotlib
import matplotlib.pyplot as plt

rnis = []
with open('build/radar-nis.txt', "r") as file:
    for line in file:
        rnis.append(float(line.split()[0]))
rnis = rnis[1:];

lnis = []
with open('build/lidar-nis.txt', "r") as file:
    for line in file:
        lnis.append(float(line.split()[0]))
lnis = lnis[1:];

fig = plt.figure()
plt.plot(rnis)
plt.plot(lnis)
plt.grid()
plt.show()
