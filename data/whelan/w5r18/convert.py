w = open('textured_mesh_comma.obj', 'w')

with open('textured_mesh.obj', 'r') as f:
    for line in f:

        if ".mtl" in line:
            newline = line.replace(".mtl", "_comma.mtl")
            w.write(newline)
        elif line[0] == "v" or line[0] == "f":
            newline = line.replace(".", ",")
            w.write(newline)
        else:
            w.write(line)

w.close()

w = open('textured_mesh_comma.mtl', 'w')

with open('textured_mesh.mtl', 'r') as f:
    for line in f:
        if line[0] == "K":
            newline = line.replace(".", ",")
            w.write(newline)
        else:
            w.write(line)

w.close()
