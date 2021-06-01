import os
import json
from helpers import quickSortByReference
import numpy as np
from math import floor
import time
from numba import jit, int32, float32
import matplotlib.pyplot as plt

plotPath = os.path.dirname(os.path.realpath(__file__)) + "/mm_plots"

class MotionMatchingDB():
    referenceTree = None

    def __init__(self, featureMatrix, featureStdDevs, featureMeans, frameTimes, animationNames, animationIDs, referenceTree = None):
        self.featureMatrix = featureMatrix
        self.featureStdDevs = featureStdDevs
        self.featureMeans = featureMeans
        self.frameTimes = frameTimes
        self.animationNames = animationNames
        self.animationIDs = animationIDs
        """leftP rightP leftV rightV hipV 20P 20D 40P 40D 60P 60D"""
        self.weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 
                                 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype='float32')
        if (referenceTree == None): self.__constructReferenceTree()
        else: self.referenceTree = referenceTree

    def __constructReferenceTree(self):
        print("Constructing reference tree...")
        featureIDs = list(range(len(self.featureMatrix)))
        self.referenceTree = self.__splitTree(featureIDs, 0)
    def __splitTree(self, featureIDs, dim):
        if (dim >= len(self.weights) or len(featureIDs) <= 2): return featureIDs # leaf node
        quickSortByReference(featureIDs, self.featureMatrix[:, dim]) # sort IDs by feature values across given dimension
        medianInd = floor((len(featureIDs) + 1) / 2)
        median = self.featureMatrix[featureIDs[medianInd]][dim]
        if (not (len(featureIDs) % 2)): # list is even
            median = (median + self.featureMatrix[featureIDs[medianInd]][dim - 1]) / 2
        left = featureIDs[:medianInd]
        right = featureIDs[medianInd:]
        return (median, self.__splitTree(left, dim + 1), self.__splitTree(right, dim + 1))
        
    def searchDBAccelerated(self, featureVector):
        frameID = self.__searchTree(self.referenceTree, featureVector, 0)
        animID = 0
        for i in range(len(self.animationIDs)):
            if (self.animationIDs[i] > frameID): break
            animID = i
        return (self.animationNames[animID], self.frameTimes[frameID], self.featureMatrix[frameID])

    def __searchTree(self, tree, featureVector, dim):
        if (type(tree) == tuple):
            if (featureVector[dim] < tree[0]): # search left branch
                return self.__searchTree(tree[1], featureVector, dim + 1)
            else: # search right branch
                return self.__searchTree(tree[2], featureVector, dim + 1)
        else: # leaf node, search references and return ID of best frame ID found
            candidateFrameID = 0
            bestDist = 100000
            for frameID in tree:
                dist = self.calculateFeatureDistance(featureVector, self.featureMatrix[frameID])
                if (dist < bestDist):
                    bestDist = dist
                    candidateFrameID = frameID
            return candidateFrameID

    def normalize(self, featureVector):
        for i in range(len(featureVector)):
            featureVector[i] = (featureVector[i] - self.featureMeans[i]) / self.featureStdDevs[i]

    def __listToTuple(data):
        if (len(data) == 3 and type(data[0]) == float): # convert to tuple
            return (data[0], MotionMatchingDB.__listToTuple(data[1]), MotionMatchingDB.__listToTuple(data[2]))
        return data
    def __convertTypes(data):
        if (type(data) == tuple):
            i1 = data[0].item()
            i2 = MotionMatchingDB.__convertTypes(data[1])
            i3 = MotionMatchingDB.__convertTypes(data[2])
            return (i1, i2, i3)
        else: return data

    def fromJSON(data):
        return MotionMatchingDB(np.array(data["features"], dtype='float32'), np.array(data["stds"], dtype='float32'), np.array(data["means"], dtype='float32'), 
                                np.array(data["times"], dtype='float32'), np.array(data["names"], dtype='S'), np.array(data["ids"], dtype='int32'), 
                                MotionMatchingDB.__listToTuple(data["tree"]))
    def toJSON(self):
        animationNames = []
        animationIDs = []
        featureStdDevs = []
        featureMeans = []
        featureMatrix = []
        frameTimes = []
        for i in self.animationNames: animationNames.append(i.decode('UTF-8'))
        for i in self.animationIDs: animationIDs.append(i.item())
        for i in self.featureStdDevs: featureStdDevs.append(i.item())
        for i in self.featureMeans: featureMeans.append(i.item())
        for i in self.featureMatrix:
            vec = []
            for j in i: vec.append(j.item())
            featureMatrix.append(vec)
        for i in self.frameTimes: frameTimes.append(i.item())
        return {"names":animationNames, "ids":animationIDs, "stds":featureStdDevs, "means":featureMeans, 
                "features":featureMatrix, "times":frameTimes, "tree":MotionMatchingDB.__convertTypes(self.referenceTree)}

    """leftP rightP leftV rightV hipV 20P 20D 40P 40D 60P 60D"""
    """0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26"""
    #includes = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26] # NOTE: re-extract if using acceleration structure
    def calculateFeatureDistance(self, featureVec1, featureVec2):
        res = 0    
        for i in range(27):
            val = abs(featureVec1[i] - featureVec2[i])
            val *= val
            res += val * self.weights[i]
        return res

    def saveTimestamps(self):
        filepath = os.path.dirname(os.path.realpath(__file__)) + "/mmtimestamps.txt"
        with open(filepath, 'w') as f:
            for i in range(len(self.timestamps)):
                f.write(str(self.timestamps[i]) + "\n")

def loadMMDB():
    filepath = os.path.dirname(os.path.realpath(__file__)) + "/motionMatchingDB.txt"
    with open(filepath) as f:
        return MotionMatchingDB.fromJSON(json.load(f))

def saveMMDB(db : MotionMatchingDB):
    filepath = os.path.dirname(os.path.realpath(__file__)) + "/motionMatchingDB.txt"
    with open(filepath, 'w') as f:
        return json.dump(db.toJSON(), f)

@jit(nopython=True)
def searchDBDirect(featureMatrix, frameTimes, weights, animationNames, animationIDs, featureVector):
    #global plotNum
    # Search DB
    startID = int32(0)
    endID = int32(0)
    candidateFrameID = int32(0)
    candidateAnimID = int32(0)
    bestDist = float32(1000000)
    #plt.clf()
    for i in range(len(animationNames)):
        startID = animationIDs[i]
        endID = animationIDs[i + 1] if ((i + 1) < len(animationIDs)) else len(featureMatrix)
        for frameID in range(startID, endID):
            dist = float32(np.sum(np.multiply(np.square(featureVector - featureMatrix[frameID]), weights)))
            #if (frameID > 1): plt.scatter(frameID, dist, c="#000000", marker="_")
            if (dist < bestDist):
                candidateFrameID = frameID
                candidateAnimID = i
                bestDist = dist
    """plt.scatter(candidateFrameID, bestDist, c="#ffbd00", marker="_")
    plt.xlabel("database index (ordered by fixed animation time)")
    plt.ylabel("feature vector distances (actual comparison)")
    plt.plot()
    plt.savefig(plotPath + "/search_actual_" + str(plotNum) + ".png", dpi=150)
    plotFeatureValues(featureMatrix, featureVector, candidateFrameID)
    plotFeatureMagnitudes(featureMatrix, featureVector, candidateFrameID)
    plotFeatureDistances(featureMatrix, featureVector, candidateFrameID)"""
    
    #plotNum += 1

    return (animationNames[candidateAnimID], frameTimes[candidateFrameID], featureMatrix[candidateFrameID])

plotNum = 1
def plotFeatureValues(m, v, c):
    global plotNum
    labels = ["left foot pos X", "left foot pos Y", "left foot pos Z", "right foot pos X", "right foot pos Y", "right foot pos Z",
              "left foot vel X", "left foot vel Y", "left foot vel Z", "right foot vel X", "right foot vel Y", "right foot vel Z",
              "hips pos X", "hips pos Y", "hips pos Z",
              "traj t+20 pos X", "traj t+20 pos Y", "traj t+20 dir X", "traj t+20 dir Y",
              "traj t+40 pos X", "traj t+40 pos Y", "traj t+40 dir X", "traj t+40 dir Y",
              "traj t+60 pos X", "traj t+60 pos Y", "traj t+60 dir X", "traj t+60 dir Y"]
    ids = list(range(m.shape[0]))
    plt.clf()
    fig, axs = plt.subplots(27, figsize=(20,7))
    for i in range(len(labels)):
        axs[i].scatter(ids[2:], m[2:,i], s=4, c="#000000", marker="_")
        axs[i].scatter(c, m[c,i], s=8, c="#ffbd00", marker="_")
        axs[i].plot(ids[2:], [v[i]] * (len(ids) - 2), c="#0070ff", linewidth=0.5)
        #axs[i].scatter(ids, [v[i]] * len(ids), s=1, c="#0070ff", marker="_")
        axs[i].set_ylabel(labels[i], rotation="horizontal", ha="right")
        if (i < len(labels) - 1): axs[i].set_xticklabels([])
        axs[i].set_yticklabels([])
    fig.set_size_inches(12, 4.8)
    plt.suptitle("Motion matching search " + str(plotNum) + " (sample clip)")
    plt.xlabel("database index (ordered by fixed animation time)")
    plt.ylabel("feature values (not normalized)")
    plt.tight_layout()
    plt.savefig(plotPath + "/search_vals_" + str(plotNum) + ".png", dpi=300)
    print("Saved feature-value plot " + str(plotNum))
def plotFeatureMagnitudes(m, v, c):
    ids = list(range(m.shape[0]))
    plt.clf()
    plt.scatter(ids[2:], np.sqrt(np.sum(np.square(m[2:]), 1)), c="#000000", marker="_")
    plt.scatter(c, np.sqrt(np.sum(np.square(m[c]))), c="#ffbd00", marker="_")
    plt.plot(ids[2:], [np.sqrt(np.sum(np.square(v)))] * (len(ids) - 2), c="#0070ff", linewidth=0.5)
    plt.suptitle("Motion matching search " + str(plotNum) + " (sample clip)")
    plt.xlabel("database index (ordered by fixed animation time)")
    plt.plot()
    plt.savefig(plotPath + "/search_mags_" + str(plotNum) + ".png", dpi=150)
    print("Saved feature-magnitude plot " + str(plotNum))
def plotFeatureDistances(m, v, c):
    ids = list(range(m.shape[0]))
    plt.clf()
    plt.scatter(ids[2:], np.sqrt(np.sum(np.square(m[2:] - v), 1)), c="#000000", marker="_")
    plt.scatter(c, np.sqrt(np.sum(np.square(m[c] - v))), c="#ffbd00", marker="_")
    #plt.plot(ids[2:], [np.sqrt(np.sum(np.square(v)))] * (len(ids) - 2), c="#0070ff", linewidth=0.5)
    plt.suptitle("Motion matching search " + str(plotNum) + " (sample clip)")
    plt.xlabel("database index (ordered by fixed animation time)")
    plt.plot()
    plt.savefig(plotPath + "/search_dists_" + str(plotNum) + ".png", dpi=150)
    print("Saved feature distance plot " + str(plotNum))
