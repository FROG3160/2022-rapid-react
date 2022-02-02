import wpilib
from wpilib import Joystick, XboxController
from .common import remap
from networktables import NetworkTables 
from magicbot import feedback



class FROGStick(Joystick):

    DEADBAND = 0.15
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1.6
    ROTATION_MIN = 0
    ROTATION_MAX = 0.5
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, channel):

        super().__init__(channel)
        self.setThrottleChannel(3)
        self.setTwistChannel(2)
        self.button_latest = {}
        self.timer = wpilib.Timer
        self.nt = NetworkTables.getTable("FROGStick_values")

    def update_NT(self, control, value):
        self.nt.putNumber(control, value)

    @feedback
    def get_speed(self):
        # Dampens the -1 to 1 values of the joystick to provide a smoothed acceleration
        speed = self.getY()
        speed = -1 * (
            speed ** 3 / self.SPEED_DIVISOR if abs(speed) > self.DEADBAND else 0
        )
        self.update_NT('speed', speed)
        return speed

    @feedback
    def get_throttle(self):
        val = super().getThrottle()
        self.update_NT('throttle', val)
        return val

    @feedback
    def get_rotation(self):
        return (
            self.getTwist() / self.ROTATION_DIVISOR
            if abs(self.getTwist()) > self.DEADBAND
            else 0
        )

    def getRangedCubedRotation(self):
        return remap(
            self.getTwist() ** 3,
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,
        )

    def getRangeRotation(self):
        return remap(
            self.getTwist(),
            self.SPEED_DIVISOR,
            1,
            self.ROTATION_MIN,
            self.ROTATION_MAX,
        )

    def get_button(self, num):
        val = self.getRawButton(num)
        self.update_NT('button_{}'.format(num), val)
        return val

    def get_debounced_button(self, num):
        """Returns the value of the joystick button. If the button is held down, then
        True will only be returned once every ``debounce_period`` seconds"""
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True
        self.update_NT('button_{}'.format(num), val)
        return val

class FROGBoxSimplicity(wpilib.XboxController):
    DEADBAND = 0.1
    SPEED_DIVISOR = 1
    ROTATION_DIVISOR = 1
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, channel):

        super().__init__(channel)
        self.button_latest = {}
        self.timer = wpilib.Timer
        self.nt = NetworkTables.getTable("FROGBoxSimplicity_values")

    @feedback
    def get_speed(self):
        speed = self.getLeftY 
        speed = -1 * (
            speed ** 3 / self.SPEED_DIVISOR if abs(speed) > self.DEADBAND
            else 0
        )

    def get_debounced_button(self, num):
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getRawButton(num):
            if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest[num] = now
                val = True
        self.update_nt('button_{}'.format(num), val)
        return val

    def update_nt(self, key, value):
        self.nt.putNumber(key, value)

class FROGBoxGunner(wpilib.XboxController):
    DEADBAND = 0.1
    ELEVATION_DIVISOR = 1
    ROTATION_DIVISOR = 1
    DEBOUNCE_PERIOD = 0.5

    def __init__(self, channel):

        super().__init__(channel)
        self.button_latest = {}
        self.timer = wpilib.Timer
        self.nt = NetworkTables.getTables("compenents/driverstation?gunner_stick")

    @feedback(key='Elevation')
    def get_elevation(self):
        return (
            self.getRightY / self.ELEVATION_DIVISOR
            if abs(self.getRightX) > self.DEADBAND 
            else 0
        )

    def get_debounced_button(self, num):
        val = False
        now = self.timer.getFPGATimestamp()
        if self.getButton(num):
             if (now - self.button_latest.get(num, 0)) > self.DEBOUNCE_PERIOD:
                 self.button_latest[num] = now
                 val = True
        self.update_nt('button_{}'.format(num), val)
        return val 

    def get_debounced_POV(self):
        val = -1
        now = self.timer.getFPGATimestamp()
        pov = self.getPOV()
        if pov > -1:
            if (now - self.button_latest.get('POV', 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest['POV'] = now
                val = pov
        if (now - self.button_latest.get('POV', 0)) < self.DEBOUNCE_PERIOD:
            self.setRumble(RIGHT, 1)
        else:
            self.setRumble(RIGHT, 0)
        self.update_nt('button_pov', val)
        return val

    def update_nt(self, key, value):
        self.nt.putNumber(key, value)
