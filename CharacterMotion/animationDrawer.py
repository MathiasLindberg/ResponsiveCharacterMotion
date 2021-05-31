from vectors import Vector3, Vector2
import pybullet as p
import time
import numpy as np
import transformations as transform
import helpers
import math


class AnimationDrawer:
    mouseDown = False
    altDown = False
    prevLinePos = (0, 0)
    startPos = (0, 0)
    prevLineTime = 0
    lineID = 0
    isFirstPosition = True
    pathAdditionTimeInterval = 0.1
    plane = -1
    pathLength = 0
    splineSegmentSteps = 3
    sphereShape = None
    pathLineCount = 0
    

    def __init__(self, plane):
        self.posLines = []
        self.dirLines = []
        self.points = []
        self.positions = []
        self.timeTableFrom = []
        self.timeTableTo = []
        self.plane = plane
        self.sphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=(1, 1, 0, 1))

    def getSpline(self, p0, p1, p2, p3):
        def spline(t):
            tt = t * t
            ttt = tt * t
            a0 = 1.0 - 3.0 * t + 3.0 * tt - ttt
            a1 = 4.0 - 6.0 * tt + 3.0 * ttt
            a2 = 1.0 + 3.0 * t + 3.0 * tt - 3.0 * ttt
            return ((a0 * p0[0] + a1 * p1[0] + a2 * p2[0] + ttt * p3[0]) / 6, (a0 * p0[1] + a1 * p1[1] + a2 * p2[1] + ttt * p3[1]) / 6)
        return spline
    def getSplineDerived(self, p0, p1, p2, p3):
        def splineDer(t):
            tt = t * t
            a0 = -3.0 * (t - 1.0)**2
            a1 = 3.0 * t * (3.0 * t - 4.0)
            a2 = -9.0 * tt + 6.0 * t + 3.0
            a3 = 3.0 * tt
            return ((a0 * p0[0] + a1 * p1[0] + a2 * p2[0] + a3 * p3[0]) / 6, (a0 * p0[1] + a1 * p1[1] + a2 * p2[1] + a3 * p3[1]) / 6)
        return splineDer

    # NOTE: length must be in normalized form
    def lengthToParametric(self, l):
        prevLength = 0
        for i in range(len(self.timeTableFrom)):
            length = self.timeTableFrom[i]
            if (length > l):
                prevParametric = 0 if (i == 0) else self.timeTableTo[i - 1]
                parametric = self.timeTableTo[i]
                pct = (l - prevLength) / (length - prevLength)
                return prevParametric + pct * (parametric - prevParametric)
            prevLength = length
        return 1

    def raycast(x, y):
        cam = p.getDebugVisualizerCamera()
        viewMat = np.reshape(cam[2], (4,4))
        projMat = np.reshape(cam[3], (4,4))
        viewToWorldMat = transform.inverse_matrix(np.dot(viewMat, projMat))
        mouseX = x / cam[0] * 2.0 - 1.0
        mouseY = 1.0 - y / cam[1] * 2.0
        mouseFromVec = [mouseX, mouseY, -1, 1]
        mouseToVec = [mouseX, mouseY, 1, 1]
        mouseWorldFromPos = np.dot(mouseFromVec, viewToWorldMat)
        mouseWorldFromPos = np.multiply(mouseWorldFromPos, 1.0 / mouseWorldFromPos[3])[0:3]
        mouseWorldToPos = np.dot(mouseToVec, viewToWorldMat)
        mouseWorldToPos = np.multiply(mouseWorldToPos, 1.0 / mouseWorldToPos[3])[0:3]
        cast = p.rayTest(mouseWorldFromPos, mouseWorldToPos)[0]
        return cast

    def addToPath(self, position):
        if (self.pathLength >= len(self.points)): self.points.append(p.createMultiBody(0, baseVisualShapeIndex=self.sphereShape))
        p.resetBasePositionAndOrientation(self.points[self.pathLength], (position[0], position[1], 0.1), (0,0,0,1))
        #p.addUserDebugLine((self.prevLinePos[0], self.prevLinePos[1], 0.1), (position[0], position[1], 0.1), [1,0,0], 5, replaceItemUniqueId=self.posLines[self.lineID])
        if (self.pathLength >= len(self.positions)): self.positions.append(position)
        else: self.positions[self.pathLength] = position
        self.pathLength += 1

    def clearPath(self):
        for i in range(self.pathLength):
            p.resetBasePositionAndOrientation(self.points[i], (0, 0, -5), (0,0,0,1))
            self.positions[i] = (0,0)
        if (self.pathLineCount != 0):
            for i in range(self.pathLength - 3):
                p.addUserDebugLine([0,0,0], [0,0,0], replaceItemUniqueId=self.dirLines[i])
            for i in range(self.pathLineCount):
                p.addUserDebugLine([0,0,0], [0,0,0], replaceItemUniqueId=self.posLines[i])
        self.pathLength = 0
        self.pathLineCount = 0

    def genPath(self):
        totalLength = 0
        self.timeTableFrom.clear()
        self.timeTableTo.clear()
        self.timeTableFrom.append(0)
        self.timeTableTo.append(0)
        prevPos = None
        first = True
        self.pathLineCount = 0
        for i in range(self.pathLength - 3):
            spline = self.getSpline(self.positions[i], self.positions[i + 1], self.positions[i + 2], self.positions[i + 3])
            splineDer = self.getSplineDerived(self.positions[i], self.positions[i + 1], self.positions[i + 2], self.positions[i + 3])
            for j in range(self.splineSegmentSteps):
                t = (j + 1.0) / self.splineSegmentSteps
                pos = spline(t)
                if (first): first = False
                else:
                    totalLength += math.sqrt((pos[0] - prevPos[0])**2 + (pos[1] - prevPos[1])**2)
                    self.timeTableFrom.append(totalLength)
                    self.timeTableTo.append(i + t)
                    if (self.pathLineCount >= len(self.posLines)): self.posLines.append(p.addUserDebugLine((prevPos[0], prevPos[1], 0.1), (pos[0], pos[1], 0.1), [0,0,0], 5))
                    else: p.addUserDebugLine((prevPos[0], prevPos[1], 0.1), (pos[0], pos[1], 0.1), [0,0,0], 5, replaceItemUniqueId=self.posLines[self.pathLineCount])
                    self.pathLineCount += 1
                prevPos = pos
            p0 = spline(0)
            d0 = splineDer(0)
            d0Mag = math.sqrt(d0[0]**2 + d0[1]**2)
            if (i >= len(self.dirLines)): self.dirLines.append(p.addUserDebugLine([p0[0], p0[1], 0.1], [p0[0] + d0[0] / d0Mag, p0[1] + d0[1] / d0Mag, 0.1], [0,0,1], 3))
            else: p.addUserDebugLine([p0[0], p0[1], 0.1], [p0[0] + d0[0] / d0Mag, p0[1] + d0[1] / d0Mag, 0.1], [0,0,1], 3, replaceItemUniqueId=self.dirLines[i])
        if (totalLength > 0):
            for i in range(len(self.timeTableFrom)):
                self.timeTableFrom[i] /= totalLength

    def getTrajectory(self, frameTime, segmentFrameTime, stepSize=20):
        trajectory = []
        if (self.pathLength < 4): return (None, None, True)
        totalTime = segmentFrameTime * (self.pathLength - 3)
        endOfPath = False
        for i in range(4):
            t = frameTime + stepSize * i
            if (t > totalTime):
                endOfPath = True
                u = self.lengthToParametric(1)
            else: u = self.lengthToParametric(t / totalTime)
                
            ind = int(u)
            frac = u - ind
            p0 = self.positions[ind]
            p1 = self.positions[min(ind + 1, self.pathLength - 1)]
            p2 = self.positions[min(ind + 2, self.pathLength - 1)]
            p3 = self.positions[min(ind + 3, self.pathLength - 1)]
            spline = self.getSpline(p0, p1, p2, p3)
            splineDer = self.getSplineDerived(p0, p1, p2, p3)
            p = spline(frac)
            d = splineDer(frac)
            dMag = math.sqrt(d[0]**2 + d[1]**2)
            trajectory.append((p[0], p[1], d[0] / dMag, d[1] / dMag))
        curr = trajectory.pop(0)
        conversion = helpers.constructInverseMatrix(Vector3(curr[0], curr[1], 0), math.atan2(curr[3], curr[2]))
        return (trajectory, conversion, endOfPath)

        """
        tClamped = helpers.clamp(t, 0, len(self.path) - 1)
        currPos = Vector2.fromlist(self.path[tClamped]).Vector3()
        currDir  = Vector2.fromlist(self.directions[tClamped]).Vector3()
        conversion = helpers.constructInverseMatrix(currPos, math.atan2(currDir.y, currDir.x))
        for i in range(t + stepSize, t + (stepSize * 4), stepSize):
            nextPos = self.path[helpers.clamp(i, 0, len(self.path) - 1)]
            nextDir = self.directions[helpers.clamp(i, 0, len(self.path) - 1)]
            trajectory.append((nextPos[0], nextPos[1], nextDir[0], nextDir[1]))
        return (trajectory, conversion)"""

    def update(self, keyEvents, mouseEvents):
        if (p.B3G_ALT in keyEvents and keyEvents[p.B3G_ALT] & p.KEY_WAS_TRIGGERED): self.altDown = True
        if (p.B3G_ALT in keyEvents and keyEvents[p.B3G_ALT] & p.KEY_WAS_RELEASED): self.altDown = False
        for e in mouseEvents:
            if (not self.altDown):
                if (e[3] == 2):
                    self.clearPath()
                    self.isFirstPosition = True
                elif (self.mouseDown and time.time() - self.prevLineTime > self.pathAdditionTimeInterval):
                    cast = AnimationDrawer.raycast(e[1], e[2])
                    if (cast[0] == self.plane):
                        if (self.isFirstPosition):
                            self.prevLinePos = (cast[3][0], cast[3][1])
                            self.isFirstPosition = False
                            self.startPos = self.prevLinePos
                        else: self.addToPath((cast[3][0], cast[3][1]))
                        self.prevLineTime = time.time()
                if (e[4] & p.KEY_WAS_TRIGGERED):
                    self.mouseDown = True
                elif (e[4] & p.KEY_WAS_RELEASED):
                    self.mouseDown = False
                    if (e[3] == 0 and self.pathLength != 0): 
                        self.genPath()
