from navx import AHRS
from magicbot import feedback
from ctre import CANifier
from .common import Buffer
import math 

BUFFERLEN = 50

SENSORUNITS_IN_INCHES = 0.0394
SENSORUNITS_IN_FEET = 0.00328
SENSORUNITS_IN_METERS = 0.001


class FROGGyro:
    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.gyro.reset()

    @feedback(key="heading")
    def getHeading(self):
        # returns gyro heading +180 to -180 degrees
        return self.gyro.getYaw()

    def resetGyro(self):
        # sets yaw reading to 0
        self.gyro.reset()

    def execute(self):
        pass
    
    @feedback()
    def getAngle(self):
        return self.gyro.getAngle()

    def setAngle(self, angle):
        self.gyro.setAngleAdjustment(angle)

    @feedback
    def getRadiansCCW(self):
        return math.radians(self.gyro.getYaw())

class FROGdar:
    canifier: CANifier

    def __init__(self):
        self.enabled = False
        self.targetRange = None
        self.rangeBuffer = Buffer(BUFFERLEN)

    def disable(self):
        self.enabled = False

    def enable(self):
        # clear range data to get fresh information
        self.rangeBuffer.clear()
        self.enabled = True

    def isValidData(self):
        return (
            self.rangeBuffer._isValidData()
            and self.targetRange is not None
        )

    @feedback(key="sensor_raw")
    def getSensorData(self):
        errorcode, (val1, val2) = self.canifier.getPWMInput(
            CANifier.PWMChannel.PWMChannel0
        )
        return val1

    @feedback(key="sensor_buffered")
    def getBufferedSensorData(self):
        return self.targetRange

    @feedback(key="range_inches")
    def getDistance(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_INCHES

    @feedback(key="range_feet")
    def getDistanceFeet(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_FEET

    @feedback(key="range_meters")
    def getDistanceMeters(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_METERS



        
    
    def execute(self):
        if self.enabled:
            # stream data into our counter
            self.rangeBuffer.append(self.getSensorData())
            if self.rangeBuffer._getBufferLength() > 0:
                self.targetRange = self.rangeBuffer.average()
            else:
                self.targetRange = None
        else:
            self.rangeBuffer.clear()
            self.targetRange = None
