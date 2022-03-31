from ctre.led import CANdle
from ctre.led import LEDStripType
from ctre.led import (
    RainbowAnimation,
    TwinkleAnimation,
    ColorFlowAnimation,
    FireAnimation,
    SingleFadeAnimation,
    StrobeAnimation

)

FORWARD = ColorFlowAnimation.Direction.Forward
BACKWARD = ColorFlowAnimation.Direction.Backward
NUM_LEDS = 8 + 47
RAINBOW = RainbowAnimation(1, 0.9, NUM_LEDS)
TWINKLE = TwinkleAnimation(0, 225, 0, 225, 0.5, NUM_LEDS)
COLORFLOWFORWARD = ColorFlowAnimation(125, 235, 0, 0, 0.5, NUM_LEDS, FORWARD)
COLORFLOWBACKWARD = ColorFlowAnimation(125, 235, 3, 0, 0.5, NUM_LEDS, BACKWARD)
TARGETING_RED = StrobeAnimation(255, 0, 0, 0, 0.1, NUM_LEDS)
FOUND_RED = StrobeAnimation(255, 0, 0, 0, 0.5, NUM_LEDS)
TARGETING_BLUE = StrobeAnimation(0, 0, 255, 0, 0.1, NUM_LEDS)
FOUND_BLUE = StrobeAnimation(0, 0, 255, 0, 0.5, NUM_LEDS)
TARGETING_GOAL = StrobeAnimation(0, 255, 0, 0, 0.1, NUM_LEDS)
FOUND_GOAL = StrobeAnimation(0, 255, 0, 0, 0.5, NUM_LEDS)
CLIMB_EXTEND = ColorFlowAnimation(200, 0, 70, 0, .9, NUM_LEDS, FORWARD)
FIRE = FireAnimation(1, 0.5, NUM_LEDS, 0.7, 0.3)


class FROGLED:
    def __init__(self, canID):
        self.candle = CANdle(canID)
        self.candle.configLEDType(LEDStripType.GRB)
        self.candle.configBrightnessScalar(0.6)
        self.Default()

    def ColorChangeRed(self):
        self.candle.setLEDs(255, 0, 0)

    def ColorChangeBlue(self):
        self.candle.setLEDs(0, 0, 255)

    def ColorChangeGreen(self):
        self.candle.setLEDs(0, 255, 0)

    def WhenShoot(self):
        self.candle.animate(COLORFLOWFORWARD)

    def climbing(self):
        self.candle.animate(CLIMB_EXTEND)

    def WhenDoneShooting(self):
        self.candle.animate(COLORFLOWBACKWARD)

    def Default(self):
        self.Magenta()

    def targetingRed(self):
        self.candle.animate(TARGETING_RED)

    def targetingBlue(self):
        self.candle.animate(TARGETING_BLUE)

    def targetingGoal(self):
        self.candle.animate(TARGETING_GOAL)

    def foundRed(self):
        self.candle.animate(FOUND_RED)

    def foundBlue(self):
        self.candle.animate(FOUND_BLUE)

    def foundGoal(self):
        self.candle.animate(FOUND_GOAL)

    def Rainbow(self):
        self.candle.animate(RAINBOW)

    def Twinkle(self):
        self.candle.animate(TWINKLE)

    def Fire(self):
        self.candle.animate(FIRE)

    def Magenta(self):
        self.candle.setLEDs(200, 0, 70)

    def Purple(self):
        self.candle.setLEDs(153, 0, 153)

    def LightGreen(self):
        self.candle.setLEDs(51, 255, 51)

    def Mint(self):
        self.candle.setLEDs(153, 255, 255)

    def LightPink(self):
        self.candle.setLEDs(255, 153, 255)
