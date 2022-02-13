from turtle import backward, forward
from ctre.led import CANdle
from ctre.led import LEDStripType
from ctre.led import StrobeAnimation, RainbowAnimation, TwinkleAnimation, ColorFlowAnimation, FireAnimation

NUM_LEDS = 120
RAINBOW = RainbowAnimation(1, 0.5, NUM_LEDS)
STROBE = StrobeAnimation(125, 255, 3 , 0, 0.5, NUM_LEDS)
TWIMKLE = TwinkleAnimation(0, 225, 0 , 225, 0.5, NUM_LEDS, 100)
COLORFLOWFOWARD = ColorFlowAnimation(125,235,0,0,0.5,NUM_LEDS, forward)
COLORFLOWBACKWARD = ColorFlowAnimation(125,235,3,0,0.5,NUM_LEDS, backward)
FIRE = FireAnimation(1,0.5,NUM_LEDS,0.7,0.3)

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
        self.candle.animate(COLORFLOWFOWARD)

    def WhenDoneShooting(self,):
        self.candle.animate(COLORFLOWBACKWARD)
