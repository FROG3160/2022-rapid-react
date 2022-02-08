from ctypes.wintypes import RGB
from ctre.led import CANdle
from ctre import led



class FROGLED():

    def __init__(self, port : int):
        #TODO check if CANBus id is correct
        self.candle = CANdle(35)
        self.candle.configLEDType(RGB)
        self.candle.setLEDs(0, 255, 0)
 
        
        
       