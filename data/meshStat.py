import glob
import os
import itertools
from sets import Set

path = os.getcwd()
filenames = []

subpath = "/data/whelan/w5r16/"
path = path + subpath

print(path)
for file in os.listdir(path):
    if file.endswith(".obj"):
        filenames.append(file.split('.')[0])
        print(filenames)



for name in filenames:
    f = open(path + "/" + name + ".obj", 'r')
    fout = open(path + "/" + name + ".txt", 'w')
    fout.write(path + "/" + name + ".obj\n")
    count = 0
    vertices = 0
    verticesNormal = 0
    verticesTexture = 0
    faces = 0
    points = []

    Count = []
    Vertices = []
    VerticesNormal = []
    VerticesTexture = []
    Faces = []
    Points = []
    UniqueIndises = []
    PlaneSizes = []

    for line in f:
        count += 1
        if(line.startswith("v ")):
            # Need to detect if point has been seen before
            pointstring = line.split(" ")
            xyz = []
            for point in pointstring[1:]:
                xyz.append(float( point.replace(",",".") ))
            points.append(xyz)
            vertices += 1
        elif(line.startswith("vt ")):
            verticesTexture += 1
        elif(line.startswith("vn ")):
            verticesNormal += 1
        elif(line.startswith("usemtl") or line.startswith("# End of File")):
            # Started reading a new plane
            if(len(UniqueIndises) != 0):
                # Add number of unique points to PlaneSizes
                UniqueIndises.sort()
                PlaneSizes.append(len(list(UniqueIndises for UniqueIndises,_ in itertools.groupby(UniqueIndises))))
                UniqueIndises = []
        elif(line.startswith("f ")):
            faces += 1
            # Need to parse line and get vertice indices
            indices = line.split(" ")[1:]
            for i in indices:
                UniqueIndises.append(  points[int(i.split("/")[0])-1] )


    fout.write("Number of lines: " + str(count) + "\n")
    fout.write("Number of vertices " + str(vertices) + "\n")
    fout.write("Number of Texture vertices " + str(verticesTexture) + "\n")


    print("Number of lines: " + str(count))
    print("Number of vertices " + str(vertices))
    print("Number of Texture vertices " + str(verticesTexture))

    fout.write("\n")
    print(" ")


    # remove duplicate points
    points.sort()
    uniquepoints = list(points for points,_ in itertools.groupby(points))
    fout.write("Unique points: " + str(len(uniquepoints)) + "\n")
    print("Unique points: " + str(len(uniquepoints)))

    fout.write("Number of faces " + str(faces) + "\n")
    fout.write("Number of normals " + str(verticesNormal) + "\n")
    print("Number of faces " + str(faces))
    print("Number of normals " + str(verticesNormal))

    fout.write("Planes:\n")
    print("Planes")
    j = 1
    for size in PlaneSizes:
        fout.write("Plane " + str(j) + ":  " + str(size) + "\n")
        print("Plane " + str(j) + ":  " + str(size))
        j += 1

    fout.write("Summed unique points:  " + str(sum(PlaneSizes)) + "\n")
    print("Summed unique points:  " + str(sum(PlaneSizes)))
