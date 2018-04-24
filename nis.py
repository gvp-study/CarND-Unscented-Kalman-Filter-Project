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

fig = plt.figure(1)
plt.plot(rnis)
plt.ylabel('NIS value')
plt.xlabel('Time')
plt.title('Normalized Innovation Squared (NIS) for Radar')
plt.grid()
plt.plot(rnis)

fig.show()

fig2 = plt.figure(2)
plt.plot(lnis, 'b')
plt.ylabel('NIS value')
plt.xlabel('Time')
plt.title('Normalized Innovation Squared (NIS) for Laser')
plt.grid()
plt.plot(lnis)

fig2.show()
