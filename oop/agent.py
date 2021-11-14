import numpy as np
import sympy

class Robot():
    def __init__(self):
        self.velocity = np.array([0, 0]).reshape(2, 1)
        self.acceleration = np.array([0, 0]).reshape(2, 1)
        self.a_max = np.array([0, 0]).reshape(2, 1)
class FireRobot(Robot):
    def __init__(self, A_max, Radius):
        super().__init__()
        self.a_max = A_max
        self.radius = Radius
    def Heading(self):
        pass
    def Clearance(self):
        pass
    def Velocity(self):
        pass
    def Decision_DWA(self):
        pass