import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import csv
import sys
import pylab

plt.rcParams["figure.figsize"] = (20,20)

csv_file = sys.argv[1]
y = []
x = []

with open(csv_file, 'rb') as csvfile:
    reader = csv.DictReader(csvfile, fieldnames=['x', 'y', 'z', 'orientation', 'speed'])
    for row in reader:
        x.append(float(row['x']))
        y.append(float(row['y']))

plt.plot(x, y)
plt.title('Path')

pylab.savefig('path.png')
