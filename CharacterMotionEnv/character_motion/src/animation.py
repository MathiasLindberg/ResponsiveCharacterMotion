import numpy as np

# Animation data type
class Animation():
    def __init__(self, name, frameTime, data, conversionMatrix = []):
        self.name = name
        self.frameCount = len(data["Frames"])
        self.frameTime = frameTime
        self.duration = self.frameCount * self.frameTime
        self.data = data
        self.conversionMatrix = conversionMatrix

    def toJSON(self):
        return {
            "name":self.name,
            "frameTime":self.frameTime,
            "data":self.data,
            "conversionMatrix":self.conversionMatrix
        }
    def fromJSON(data):
        animation = Animation(
            data["name"], 
            data["frameTime"], 
            data["data"], 
            data["conversionMatrix"]
        )
        for i in range(len(animation.conversionMatrix)):
            animation.conversionMatrix[i] = np.array(animation.conversionMatrix[i])
        return animation