import math


class Circle:
    def __init__(self,radius,clockwise = True):
        self.radius = radius
        self.clockwise = clockwise
        self.base_width = 0.248 #Neato wheelbase
    
    def get_distance(self):
        arc = 2*math.pi*self.radius
        lwheeldist = 2*math.pip*(radius+self.base_width)/100
        rwheeldist = 2*math.pip*(radius-self.base_width)/100
        return lwheeldist, rwheeldist, arc
    