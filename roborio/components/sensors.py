from navx import AHRS
from magicbot import feedback

class FROGGyro:
    def __init__(self):
      # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.gyro.reset()

    @feedback(key='heading')
    def getHeading(self):
        # returns gyro heading +180 to -180 degrees
        return self.gyro.getYaw()

    def resetGyro(self):
        # sets yaw reading to 0
        self.gyro.reset()

    def execute(self):
        pass
