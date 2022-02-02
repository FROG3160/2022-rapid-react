# components that are part of the shooter
from roborio.components.sensors import FROGdar


class FROGShooter:
    lidar: FROGdar 

    def __init__(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def set_automatic(self):
        pass

    def set_manual(self):
        pass

    def disable(self):
        pass

    def getdistance(self):
        # get the value/distance from the lidar in inches
        return self.lidar.getDistance() 
