from ctre.led import CANdle
from ctre.led import LEDStripType


class FROGLED():

    def __init__(self, port : int):
        self.candle = CANdle(port)
        self.candle.configLEDType(LEDStripType.RGB)
        self.candle.setLEDs(0, 255, 0)

    def ColorChangeRed(self,) :
        self.candle.setLEDs(255, 0, 0)        

    def ColorChangeBlue(self,) :
        self.candle.setLEDs(0, 0, 255)

    def WhenShoot(self,):
        self.candle.setLEDs(252, 240, 3)

    def WhenDoneShooting(self,):
        self.candle.setLEDs(190, 252, 3)
