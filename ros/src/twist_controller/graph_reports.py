import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import csv
import sys
import pylab

plt.rcParams["figure.figsize"] = (100,6)

csv_file = sys.argv[1]
y_wheel = []
y_cmd = []
y_speed = []
y_proposed_speed = []
x = []

with open(csv_file, 'rb') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        x.append(float(row['timestamp']))
        y_wheel.append(float(row['actual']))
        y_cmd.append(float(row['proposed']))
        y_speed.append(float(row['actual_speed']))
        y_proposed_speed.append(float(row['proposed_speed']))

fig, ax = plt.subplots(2,1)

plt.subplot(2,1,1)
wheel_plt, = plt.plot(x, y_wheel)
cmd_plt, = plt.plot(x, y_cmd)
ax[0].xaxis.set_major_locator(ticker.MultipleLocator(1))
plt.legend([wheel_plt, cmd_plt], ["Actual", "Proposed"])
plt.title('Steering Wheel Angle Cmd, Actual vs Proposed ')

plt.subplot(2,1,2)
speed_plt, = plt.plot(x, y_speed)
proposed_speed_plt, = plt.plot(x, y_proposed_speed)
plt.legend([speed_plt, proposed_speed_plt], ["Actual", "Proposed"])
plt.title('Speed, Actual vs Proposed ')

pylab.savefig('foo.png')
