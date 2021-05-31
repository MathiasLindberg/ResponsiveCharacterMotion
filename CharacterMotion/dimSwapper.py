import os

isJoint = False
filepath = os.path.dirname(os.path.realpath(__file__)) + "/externals/"
with open(filepath + "character.urdf") as fromFile:
    with open (filepath + "character_updated.urdf", 'w') as toFile:
        for line in fromFile.readlines():
            if (line.__contains__("<joint")):
                isJoint = True
            elif (line.__contains__("</joint")):
                isJoint = False
            elif (isJoint and line.__contains__("origin")):
                line = line.split("xyz=\"")
                xyz = line[1].split("\"")[0].split(" ")
                xyz = "xyz=\"" + str(float(xyz[0]) * 0.01) + " " + str(float(xyz[1]) * 0.01) + " " + str(float(xyz[2]) * 0.01) + "\""
                line = line[0] + xyz + "/>\n"
            toFile.write(line)