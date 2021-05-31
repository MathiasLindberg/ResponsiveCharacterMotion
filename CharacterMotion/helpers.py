import math
import numpy as np
import pybullet as p
import transformations as transform
from vectors import *

def constructInverseMatrix(position, rotation):
    r = transform.rotation_matrix(rotation, [0, 0, 1])
    t = transform.translation_matrix(position.tolist())
    return transform.inverse_matrix(transform.concatenate_matrices(t, r))

def calculateForward(pose):
    leftRightShoulderVec = pose[4] - pose[5]
    forwardVec = leftRightShoulderVec.cross(Vector3.up()).Vector2().normalize()
    return forwardVec

# get the shortest signed angle between two 2D vectors
def shortestSignedAngleBetween(vec1 : Vector2, vec2 : Vector2):
    vecDiff = rotateBy(vec2, -math.atan2(vec1.y, vec1.x))
    angle = math.acos(vecDiff.x)
    return angle if math.atan2(vecDiff.y, vecDiff.x) < 0 else -angle

# Rotate 2D vector by angle in radians
def rotateBy(vec : Vector2, rad):
    return Vector2(math.cos(rad) * vec.x - math.sin(rad) * vec.y, math.sin(rad) * vec.x + math.cos(rad) * vec.y)

def clamp(value, lower, upper): 
    return min(upper, max(value, lower))
def lerpVectors(vec1 : Vector3, vec2 : Vector3, t):
    return Vector3(lerp(vec1.x, vec2.x, t), lerp(vec1.y, vec2.y, t), lerp(vec1.z, vec2.z, t))
def lerp(val1, val2, t):
    return val1 + t * (val2 - val1)

# Sorts a list of IDs referencing to their corresponding values
def quickSortByReference(idList, values):
    __quickSort(idList, values, 0, len(idList) - 1)
# Algorithms 4. edition by Robert Sedgewick and Kevin Wayne, chapter 2, section 3
def __quickSort(ls, values, l, h):
    if (h <= l): return
    j = __qsPartition(ls, values, l, h)
    __quickSort(ls, values, l, j - 1)
    __quickSort(ls, values, j + 1, h)
def __qsPartition(ls, values, l, h):
    i = l
    j = h + 1
    v = values[ls[l]]
    while (True):
        i += 1
        j -= 1
        while (values[ls[i]] < v):
            if (i == h): break
            else: i += 1
        while (v < values[ls[j]]):
            if (j == l): break
            else: j -= 1
        if (i >= j): break
        temp = ls[i]
        ls[i] = ls[j]
        ls[j] = temp
    temp = ls[l]
    ls[l] = ls[j]
    ls[j] = temp
    return j

# Test quick sort implementation
"""
vals = list("QUICKSORTEXAMPLE")
ids = list(range(len(vals)))
quickSortByReference(ids, vals)
valsOrdered = [0] * len(ids)
for i in ids:
    valsOrdered[i] = vals[ids[i]]
print(valsOrdered == list("ACEEIKLMOPQRSTUX")) # True
valsOrdered2 = list(np.sort(vals))
print(valsOrdered == valsOrdered2) # True
"""

def capsuleBounds(length, radius):
    return [Vector3(length * -0.5 - radius, -radius, -radius), Vector3(length * 0.5 + radius, radius, radius)]
def sphereBounds(radius):
    return [Vector3(-radius, -radius, -radius), Vector3(radius, radius, radius)]
def boxBounds(width, height, depth):
    return [Vector3(width * -0.5, height * -0.5, depth * -0.5), Vector3(width * 0.5, height * 0.5, depth * 0.5)]