# To adjust targeting settings within Photon Vision, go to 10.31.60.3:5800
# in browser while on robot network.
# Expecting 15-20 fps with cargo tracking and 50-75 fps with goal tracking.
# Cargo cam resolution is 320x240. Goal cam is resolution is 640x480.

from lib2to3.pgen2.driver import Driver
import photonvision
from .common import Buffer
from wpilib import DriverStation
from magicbot import feedback

# Values by which the the results are divided to give an output of -1 to 1.
# LifeCam FOV is H62.8 x V37.9. Yaw is -31.4 to 31.4 degrees.
#   Pitch is -18.9 to 18.9 degrees.
# PiCam horizontal FOV is 53.5. Yaw is -26.75 to 26.75 degrees.
LC_X_div = 31.4
LC_Y_div = 18.9
PI_X_div = 26.7

kRed = DriverStation.Alliance.kRed  # 0
kBlue = DriverStation.Alliance.kBlue  # 1
kInvalid = DriverStation.Alliance.kInvalid  # 2


class FROGVision:
    def __init__(self):

        # Sets varible to specifed camera. Must match camera name
        # on photon vision web interface.
        self.CARGOcam = photonvision.PhotonCamera("CargoCam")
        self.Goalcam = photonvision.PhotonCamera("TargetPiCam")
        self.driverstation = DriverStation()

        # Sets the number of values held in each buffer.
        self.CargoYawBuff = Buffer(8)
        self.CargoPitchBuff = Buffer(8)
        self.GoalYawBuff = Buffer(5)
        self.allianceColor = self.driverstation.getAlliance()

    @feedback('Alliance Color')
    def getAllianceColor(self):
        return self.allianceColor

    def getCurrentCargoPipeline(self):
        return self.CARGOcam.getPipelineIndex()

    # Yaw is left- and right+ degrees with center being 0
    # Values are divided so as to make the final values between -1 and 1
    def getCargoX(self):
        if target := self.CargoTarget.getYaw():
            return target / LC_X_div

    def getCargoXAverage(self):
        if target := self.CargoYawBuff:
            return target / LC_X_div

    def getGoalX(self):
        if target := self.GoalTarget.getYaw():
            return target / PI_X_div

    def getGoalXAverage(self):
        if target := self.GoalYawBuff.average():
            return target / PI_X_div

    # Pitch is up+ and down- degrees with center being 0
    def getCargoY(self):
        if target := self.CargoTarget.getPitch():
            return target / LC_Y_div

    def getCargoYAverage(self):
        if target := self.CargoPitchBuff.average():
            return target / LC_Y_div

    def setCargoAllianceColor(self):
        self.CARGOcam.setPipelineIndex(self.allianceColor + 1)

    def setCargoBlue(self):
        self.CARGOcam.setPipelineIndex(2)

    def setCargoRed(self):
        self.CARGOcam.setPipelineIndex(1)

    @feedback("Targeted Cargo")
    def targetingAllianceCargo(self):
        return self.getCurrentCargoPipeline() == self.allianceColor + 1

    def execute(self):
        # Grabs the latest results from the Cameras and then get the
        # best target.  Best target is chosen based off of the
        # parameters on the photon vision web interface.
        self.CargoResults = self.CARGOcam.getLatestResult()
        self.CargoTarget = self.CargoResults.getBestTarget()

        self.GoalResults = self.Goalcam.getLatestResult()
        self.GoalTarget = self.GoalResults.getBestTarget()

        # Adds the most recent result to the buffers.
        self.CargoYawBuff.append(self.CargoTarget.getYaw())
        self.CargoPitchBuff.append(self.CargoTarget.getPitch())
        self.GoalYawBuff.append(self.GoalTarget.getYaw())
