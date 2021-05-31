import math

# Class for storing vector data, including helper functions
class Vector3():
    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
        self.z = z
    def __add__(self, other):
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")"
    def __sub__(self, other):
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
    def __mul__(self, value):
        return Vector3(self.x * value, self.y * value, self.z * value)
    def __truediv__(self, value):
        return Vector3(self.x / value, self.y / value, self.z / value)
    def __neg__(self):
        return Vector3(-self.x, -self.y, -self.z)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z
    def __ne__(self, other):
        return self.x != other.x and self.y != other.y and self.z != other.z

    def tolist(self):
        return [self.x, self.y, self.z]
    def totuple(self):
        return (self.x, self.y, self.z)
    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z
    def normalize(self):
        length = self.magnitude()
        return self / length
    def cross(self, other):
        return Vector3(self.y * other.z - self.z * other.y, self.z * other.x - self.x * other.z, self.x * other.y - self.y * other.x)
    def Vector2(self):
        return Vector2(self.x, self.y)
    def fromlist(ls):
        return Vector3(ls[0], ls[1], ls[2])

    def up():
        return Vector3(0, 0, 1)
    def right():
        return Vector3(1, 0, 1)
    def forward():
        return Vector3(0, 1, 0)

class Vector2():
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y
    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)
    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"
    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)
    def __mul__(self, value):
        return Vector2(self.x * value, self.y * value)
    def __truediv__(self, value):
        return Vector2(self.x / value, self.y / value)
    def __neg__(self):
        return Vector2(-self.x, -self.y)
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __ne__(self, other):
        return self.x != other.x and self.y != other.y

    def tolist(self):
        return [self.x, self.y]
    def totuple(self):
        return (self.x, self.y)
    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
    def dot(self, other):
        return self.x * other.x + self.y * other.y
    def normalize(self):
        length = self.magnitude()
        return self / length
    def Vector3(self):
        return Vector3(self.x, self.y, 0)
    def fromlist(ls):
        return Vector2(ls[0], ls[1])

    def up():
        return Vector2(0, 1)
    def right():
        return Vector2(1, 0)

def vec3To4List(vec):
    return [vec.x, vec.y, vec.z, 1]
