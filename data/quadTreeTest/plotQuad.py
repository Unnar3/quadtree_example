import glob
import os
import itertools
from sets import Set
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
import matplotlib.path as mplPath
import random

# These are the "Tableau 20" colors as RGB.
tableau20 = [(31, 119, 180), (174, 199, 232), (255, 127, 14), (255, 187, 120),
             (44, 160, 44), (152, 223, 138), (214, 39, 40), (255, 152, 150),
             (148, 103, 189), (197, 176, 213), (140, 86, 75), (196, 156, 148),
             (227, 119, 194), (247, 182, 210), (127, 127, 127), (199, 199, 199),
             (188, 189, 34), (219, 219, 141), (23, 190, 207), (158, 218, 229)]

# Scale the RGB values to the [0, 1] range, which is the format matplotlib accepts.
for i in range(len(tableau20)):
    r, g, b = tableau20[i]
    tableau20[i] = (r / 255., g / 255., b / 255.)

# You typically want your plot to be ~1.33x wider than tall. This plot is a rare
# exception because of the number of lines being plotted on it.
# Common sizes: (10, 7.5) and (12, 9)
plt.figure(figsize=(12, 14))

# Remove the plot frame lines. They are unnecessary chartjunk.
ax = plt.subplot(111)
ax.set_xlim(0,10)
ax.set_ylim(0,10)
ax.spines["top"].set_visible(False)
ax.spines["bottom"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.spines["left"].set_visible(False)
ax.get_xaxis().set_visible(False)
ax.get_yaxis().set_visible(False)
ax.set_aspect('equal', adjustable='box')


path = os.getcwd()
path = path + "/data/quadTreeTest/"
filenames = []

polystart =  []
polygonx = []
polygony = []
poly = open(path + "/" + "polygon" + ".txt", 'r')
for line in poly:
    if(len(polystart) is 0):
        polystart = [int(float(i)) for i in line.split(",")]
    else:
        tmp = [float(i) for i in line.split(",")]
        polygonx.append(tmp[0])
        polygony.append(tmp[1])

Polygonx = []
Polygony = []
for index, obj in enumerate(polystart):
    if(index != len(polystart)-1):
        Polygonx.append(polygonx[polystart[index]:polystart[index+1]])
        Polygony.append(polygony[polystart[index]:polystart[index+1]])
    else:
        Polygonx.append(polygonx[polystart[index]:])
        Polygony.append(polygony[polystart[index]:])

print(Polygonx)
print(Polygony)


print(path)
for file in os.listdir(path):
    if file.endswith(".txt"):
        filenames.append(file.split('.')[0])
        print(filenames)

f = open(path + "/" + "quad2" + ".txt", 'r')
for line in f:
    print(line)
    v = line.split(",")
    v = [float(i) for i in v]
    # for i in v:
    #     print(i)
    zo = 2
    i = 0
    a = 1;
    if(v[-1] > 1):
        i = 1
        zo = 0
        a = 0.5
    elif(v[-1] == 1):
        i = 10
        zo = 1
    plt.plot([ v[0], v[0]+v[2], v[0]+v[2], v[0], v[0] ],[ v[1], v[1], v[1]+v[2], v[1]+v[2], v[1] ], color=tableau20[i], zorder=zo, alpha = a)

for x, y in zip(Polygonx, Polygony):
    plt.plot(x,y,color=tableau20[6])

plt.savefig(path + 'quad2.png', transparent=True, bbox_inches='tight')
# plt.show()


plt.figure(figsize=(12, 12))
# Remove the plot frame lines. They are unnecessary chartjunk.
ax = plt.subplot(111)
ax.set_xlim(0,10)
ax.set_ylim(0,10)
ax.spines["top"].set_visible(False)
ax.spines["bottom"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.spines["left"].set_visible(False)
ax.get_xaxis().set_visible(False)
ax.get_yaxis().set_visible(False)
ax.set_aspect('equal', adjustable='box')

triangles = open(path + "triangles" + ".txt", 'r')

Trianglesx = []
Trianglesy = []
Trianglesvert = []

P = True
for line in triangles:
    if(line.startswith("# Points")):
        P = True
    elif(line.startswith("# Vertices")):
        P = False
    elif(P is True):
        Trianglesx.append(float(line.split(",")[0]))
        Trianglesy.append(float(line.split(",")[1]))
    else:
        Trianglesvert.append( [int(float(i)) for i in line.split(",")] )

for vert in Trianglesvert:
    tmpx = [Trianglesx[vert[-1]]]
    tmpy = [Trianglesy[vert[-1]]]
    for v in vert:
        tmpx.append( Trianglesx[v]  )
        tmpy.append( Trianglesy[v]  )
    plt.plot(tmpx,tmpy, color = tableau20[0])

# plt.scatter(Trianglesx, Trianglesy)
plt.savefig(path + 'quad3.png', transparent=True, bbox_inches='tight')
# plt.show()


fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111)
paths = []

for (polx,poly) in zip(Polygonx,Polygony):
    bbPath = mplPath.Path( np.array( zip(polx, poly) ) )
    patch = patches.PathPatch(bbPath, edgecolor=tableau20[6], fill=False, lw=3)
    paths.append(bbPath)
    ax.add_patch(patch)

rangeX = (0, 10)
rangeY = (0, 10)
pointx = []
pointy = []
count = 0
while count < 2000:
    x = random.uniform(0.01, 9.99)
    y = random.uniform(0.01, 9.99)
    if(paths[0].contains_point((x,y))):
        print("hmmm")
        if not( paths[1].contains_point((x,y))):
            pointx.append(x)
            pointy.append(y)
            count += 1

plt.plot(pointx, pointy, "o", markeredgecolor=tableau20[10], alpha=1, ms = 2)

ax.set_xlim(0,10)
ax.set_ylim(0,10)
ax.spines["top"].set_visible(False)
ax.spines["bottom"].set_visible(False)
ax.spines["right"].set_visible(False)
ax.spines["left"].set_visible(False)
ax.get_xaxis().set_visible(False)
ax.get_yaxis().set_visible(False)
plt.savefig(path + 'quad1.png', transparent=True, bbox_inches='tight')
plt.show()
