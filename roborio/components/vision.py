# To adjust targeting settings within Photon Vision, go to 10.31.60.3:5800
# in browser while on robot network.
# Expecting 15-20 fps with cargo tracking and 50-75 fps with goal tracking.
# Cargo cam resolution is 320x240. Goal cam is resolution is 640x480.

import photonvision
from photonvision import PhotonUtils
from components.common import Buffer
from wpilib import DriverStation
from magicbot import feedback, tunable
from wpimath.filter import MedianFilter

# Values by which the the results are divided to give an output of -1 to 1.
# LifeCam FOV is H62.8 x V37.9. Yaw is -31.4 to 31.4 degrees.
#   Pitch is -18.9 to 18.9 degrees.
# PiCam horizontal FOV is 53.5. Yaw is -26.75 to 26.75 degrees.
LC_X_div = 62.8 / 2
LC_Y_div = 37.9 / 2
PI_X_div = 53.5 / 2
PI_Y_div = 41.41 / 2

FILTER_RESET_COUNT = 5
FILTER_SIZE = 12  # 12 samples at 20 ms each sample = 240ms of samples

TARGET_HEIGHT_METERS = 2.6416  # 8ft. 8in.  #TODO: Confirm this value
CAMERA_HEIGHT_METERS = 0.46482  # 18.3 inches
CAMERA_PITCH_RADIANS = 0.4658284  # 26.69 degrees

kRed = DriverStation.Alliance.kRed  # 0
kBlue = DriverStation.Alliance.kBlue  # 1
kInvalid = DriverStation.Alliance.kInvalid  # 2


class FROGVision:

    goal_offset = tunable(1.2)

    def __init__(self):

        # Sets varible to specifed camera. Must match camera name
        # on photon vision web interface.
        self.CARGOcam = photonvision.PhotonCamera("CargoCam")
        self.Goalcam = photonvision.PhotonCamera("TargetPiCam")
        self.driverstation = DriverStation

        # Sets the number of values held in each buffer.
        self.CargoYawBuff = Buffer(8)
        self.CargoPitchBuff = Buffer(8)
        self.GoalYawBuff = Buffer(5)
        self.allianceColor = self.driverstation.getAlliance()

        self.deactivateDriverMode()

        self.CargoXFilter = MedianFilter(FILTER_SIZE)
        self.GoalXFilter = MedianFilter(FILTER_SIZE)
        self.CargoYFilter = MedianFilter(FILTER_SIZE)
        self.GoalYFilter = MedianFilter(FILTER_SIZE)

        self.hasGoalTargets = False
        self.hasCargoTargets = False
        self.filterGoalResetCount = 0
        self.filterCargoResetCount = 0

        self.filteredCargoYaw = None
        self.filteredCargoPitch = None
        self.filteredGoalYaw = None
        self.filteredGoalPitch = None

    def activateDriverMode(self):
        self.CARGOcam.setDriverMode(True)

    def deactivateDriverMode(self):
        self.CARGOcam.setDriverMode(False)

    def getAllianceColor(self):
        return self.allianceColor

    def getCurrentCargoPipeline(self):
        return self.CARGOcam.getPipelineIndex()

    # Yaw is left- and right+ degrees with center being 0
    # Values are divided so as to make the final values between -1 and 1
    def getCargoX(self):
        if target := self.CargoTarget.getYaw():
            return target / LC_X_div

    @feedback(key="Cargo X")
    def getCargoXAverage(self):
        if target := self.CargoYawBuff.average():
            return target / LC_X_div

    @feedback()
    def getFilteredCargoYaw(self):
        return self.filteredCargoYaw

    @feedback()
    def getFilteredCargoPitch(self):
        return self.filteredCargoPitch

    @feedback()
    def get_FilteredGoalYaw(self):
        return self.filteredGoalYaw

    @feedback()
    def getFilteredGoalPitch(self):
        return self.filteredGoalPitch

    @feedback()
    def getRangeInches(self):
        if self.filteredGoalPitch:
            return (PhotonUtils.calculateDistanceToTarget(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                self.filteredGoalPitch,
            ) * 5000) / 127

    def getGoalX(self):
        if target := self.GoalTarget.getYaw():
            return (target / PI_X_div) + self.goal_offset

    @feedback(key="Goal X")
    def getGoalXAverage(self):
        if target := self.GoalYawBuff.average():
            return target / PI_X_div

    # Pitch is up+ and down- degrees with center being 0
    def getCargoY(self):
        if target := self.CargoTarget.getPitch():
            return target / LC_Y_div

    @feedback(key="Cargo Y")
    def getCargoYAverage(self):
        if target := self.CargoPitchBuff.average():
            return target / LC_Y_div

    def getDriverMode(self):
        self.CARGOcam.getDriverMode()

    def resetCargoFilters(self):
        self.CargoXFilter.reset()
        self.CargoYFilter.reset()
        self.filteredCargoYaw = None
        self.filteredCargoPitch = None

    def resetGoalFilters(self):
        self.GoalXFilter.reset()
        self.GoalYFilter.reset()
        self.filteredGoalYaw = None
        self.filteredGoalPitch = None

    def setCargoAllianceColor(self):
        self.CARGOcam.setPipelineIndex(self.allianceColor + 1)

    def setCargoBlue(self):
        self.CARGOcam.setPipelineIndex(2)

    def setCargoRed(self):
        self.CARGOcam.setPipelineIndex(1)

    @feedback(key="Alliance Cargo")
    def targetingAllianceCargo(self):
        return self.getCurrentCargoPipeline() == self.allianceColor.value + 1

    def updateCargoFilters(self):
        self.filteredCargoYaw = self.CargoXFilter.calculate(
            self.CargoTarget.getYaw()
        )
        self.filteredCargoPitch = self.CargoYFilter.calculate(
            self.CargoTarget.getPitch()
        )

    def updateGoalFilters(self):
        print("set X filter: ", self.GoalTarget.getYaw())
        self.filteredGoalYaw = self.GoalXFilter.calculate(
            self.GoalTarget.getYaw()
        )
        self.filteredGoalPitch = self.GoalYFilter.calculate(
            self.GoalTarget.getPitch()
        )
        print("Filtered: ", self.filteredGoalYaw, self.filteredGoalPitch)

    def execute(self):
        # Grabs the latest results from the Cameras and then get the
        # best target.  Best target is chosen based off of the
        # parameters on the photon vision web interface.

        self.CargoResults = self.CARGOcam.getLatestResult()
        if self.CargoResults.hasTargets():
            self.hasCargoTargets = True
            self.CargoTarget = self.CargoResults.getBestTarget()
            # Adds the most recent result to the buffers.
            self.CargoYawBuff.append(self.CargoTarget.getYaw())
            self.CargoPitchBuff.append(self.CargoTarget.getPitch())
            self.updateCargoFilters()
        else:
            self.hasCargoTargets = False
            self.filterCargoResetCount += 1
            if self.filterCargoResetCount == FILTER_RESET_COUNT:
                self.resetCargoFilters()
                self.filterCargoResetCount = 0

        self.GoalResults = self.Goalcam.getLatestResult()
        if self.GoalResults.hasTargets():
            self.hasGoalTargets = True
            self.GoalTarget = self.GoalResults.getBestTarget()
            self.GoalYawBuff.append(self.GoalTarget.getYaw())
            self.updateGoalFilters()
        else:
            self.hasGoalTargets = False
            self.filterGoalResetCount += 1
            if self.filterGoalResetCount == FILTER_RESET_COUNT:
                self.resetGoalFilters()
                self.filterGoalResetCount = 0
