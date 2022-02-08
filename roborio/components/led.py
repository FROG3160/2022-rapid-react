from wpilib import AddressableLED

LEDData = AddressableLED.LEDData


class FROGLED(AddressableLED):
    def __init__(self, pwmChannel, ledCount):
        super().__init__(pwmChannel)
        # set the number of leds
        self.ledCount = ledCount
        self.setLength(ledCount)
        self.setMagenta()
        self.start()

    def fillRGB(self, RGBTuple):
        self.data = [LEDData(*RGBTuple) for _ in range(self.ledCount)]
        self.apply()

    def setMagenta(self):
        # flood fill magenta
        self.fillRGB((255, 0, 255))

    def setFROGGreen(self):
        self.fillRGB((0, 217, 54))

    def setRed(self):
        self.fillRGB((255, 0, 0))

    def setBlue(self):
        self.fillRGB((0, 0, 255))

    def setGreen(self):
        self.fillRGB((0, 255, 0))

    def setRainbow(self):
        rainbow = i = 0
        for d in self.data:
            hue = rainbow + ((i * 180 / self.ledCount) % 180)
            d.setHSV(int(hue), 255, 128)
            LEDData
            i += 1
            if rainbow > 180:
                rainbow = 0
            rainbow += 3
        self.apply()

    def apply(self):
        self.setData(self.data)
        
class FROGStrip(AddressableLED):
    def __init__(self, pwmChannel: int) -> None:
        super().__init__(pwmChannel)