import pybullet as p
import pybullet_data
from character import *
import time

class Environment():
    def __init__(self, args):
        # Setup arguments
        self.useGUI = "useGUI" in args and args["useGUI"]
        self.usePlane = "usePlane" in args and args["usePlane"]
        self.timeStep = (1. / 600.) if "timeStep" not in args else args["timeStep"]
        self.solverIterations = 30 if "solverIterations" not in args else args["solverIterations"]
        self.gravity = -10 if "upGravity" not in args else args["upGravity"]
        self.globalTime = 0.0
        self.startTime = time.time()
        # Connect PyBullet
        p.connect(p.GUI if self.useGUI else p.DIRECT)
        # Set parameters
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(numSolverIterations=self.solverIterations)
        p.setPhysicsEngineParameter(fixedTimeStep=self.timeStep)
        p.setGravity(0, 0, self.gravity)
        if (self.usePlane): 
            self.planeID = p.loadURDF("plane.urdf")
            p.resetBasePositionAndOrientation(self.planeID, [0,0,0], [0,0,0,1])
            p.setCollisionFilterGroupMask(self.planeID, -1, 1, 0)

        self.characters = []
        
    def stepSimulation(self, updateCharacters=True):
        self.globalTime = time.time() - self.startTime
        if (updateCharacters):
            for character in self.characters:
                Character.stepCharacter(character, self.globalTime)
        p.stepSimulation()
        
    def resetTime(self):
        self.startTime = time.time()
        self.globalTime = 0
        