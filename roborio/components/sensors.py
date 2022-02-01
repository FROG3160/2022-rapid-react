import wpilib 
#  Resolve issue of magicbot not being defined
from magicbot import feedback
from ctre import CANifier
from .common import Buffer

BUFFERLEN = 50

SENSORUNITS_IN_INCHES = 0.0394

class FROGdar:
    pwm_sensor: CANifier

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
            self.rangeBuffer.lengthFiltered() >= BUFFERLEN
            and self.targetRange is not None
        )

    @feedback(key='sensor_raw') 
    def getSensorData(self):
        errorcode, (val1, val2) = self.pwm_sensor.getPWMInput(
            CANifier.PWMChannel.PWMChannel0
        )
        return val1

    @feedback(key='sensor_buffered')
    def getBufferedSensorData(self):
        return self.targetRange

       
        
    @feedback(key="range_inches")
    def getDistance(self):
        if self.isValidData():
            return self.getBufferedSensorData() * SENSORUNITS_IN_INCHES

    def execute(self):
        if self.enabled:
            # stream data into our counter
            self.rangeBuffer.append(self.getSensorData())
            if self.rangeBuffer.lengthFiltered() > 0:
                self.targetRange = self.rangeBuffer.average()
            else:
                self.targetRange = None
        else:
            self.rangeBuffer.clear()
            self.targetRange = None

