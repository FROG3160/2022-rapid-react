# To adjust targeting settings within Photon Vision, go to 10.31.60.3:5800 in browser while on robot network.
# Expecting 15-20 fps with cargo tracking and 50-75 fps with goal tracking.
# Cargo cam resolution is 320x240. Goal cam is resolution is 640x480.

import photonvision
from .common import Buffer
from wpilib import DriverStation
from magicbot import feedback

# Values by which the the results are divided to give an output of -1 to 1.
# LifeCam FOV is H62.8 x V37.9. Yaw is -31.4 to 31.4 degrees. Pitch is -18.9 to 18.9 degrees.
# PiCam horizontal FOV is 53.5. Yaw is -26.75 to 26.75 degrees.
LC_X_div = 31.4
LC_Y_div = 18.9
PI_X_div = 26.7

kRed = DriverStation.Alliance.kRed
kBlue = DriverStation.Alliance.kBlue
kInvalid = DriverStation.Alliance.kInvalid


class FROGVision:
    def __init__(self):

        # Sets varible to specifed camera. Must match camera name on photon vision web interface.
        self.CARGOcam = photonvision.PhotonCamera("CargoCam")
        self.Goalcam = photonvision.PhotonCamera("TargetPiCam")

        # Sets the number of values held in each buffer.
        self.CargoYawBuff = Buffer(8)
        self.CargoPitchBuff = Buffer(8)
        self.GoalYawBuff = Buffer(5)
        self.allianceColor = kInvalid

    def setup(self):

        # Switches to specified pipeline depending on alliance color.
        # 2 is Blue Cargo, 1 is Red Cargo, and -1 is Driver Mode.

        if self.allianceColor == kBlue:
            self.CARGOcam.setPipelineIndex(2)

        elif self.allianceColor == kRed:
            self.CARGOcam.setPipelineIndex(1)

        else:
            self.CARGOcam.setDriverMode(True)

    def execute(self):

        self.getAllianceColor()
        # Grabs the latest results from the Cameras and then get the best target.
        # Best target is chosen based off of the parameters on the photon vision web interface.
        self.CargoResults = self.CARGOcam.getLatestResult()
        self.CargoTarget = self.CargoResults.getBestTarget()

        self.GoalResults = self.Goalcam.getLatestResult()
        self.GoalTarget = self.GoalResults.getBestTarget()

        # Adds the most recent result to the buffers.
        self.CargoYawBuff.append(self.CargoTarget.getYaw())
        self.CargoPitchBuff.append(self.CargoTarget.getPitch())
        self.GoalYawBuff.append(self.GoalTarget.getYaw())

    def CurrentCargoPipeline(self):
        return self.CARGOcam.getPipelineIndex()

    # Yaw is left- and right+ degrees with center being 0
    # Values are divided so as to make the final values between -1 and 1
    def getGoalX(self):
        return self.GoalTarget.getYaw() / PI_X_div

    def getGoalXAverage(self):
        return self.GoalYawBuff.average() / PI_X_div

    def getCargoX(self):
        return self.CargoTarget.getYaw() / LC_X_div

    def getCargoXAverage(self):
        return self.CargoYawBuff.average() / LC_X_div

    # Pitch is up+ and down- degrees with center being 0
    def getCargoY(self):
        return self.CargoTarget.getPitch() / LC_Y_div

    def getCargoYAverage(self):
        return self.CargoPitchBuff.average() / LC_Y_div

    def setCargoRed(self):
        return self.CARGOcam.setPipelineIndex(1)

    def setCargoBlue(self):
        return self.CARGOcam.setPipelineIndex(2)

    def setDriverMode(self):
        return self.CARGOcam.setDriverMode(True)

    def setAllianceColor(self, color):
        self.allianceColor = color
        self.setup()

    @feedback()
    def getAllianceColor(self):
        return self.allianceColor
