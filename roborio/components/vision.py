import photonvision


class FROGVision:

    def __init__(self):

        # Sets varible to specifed camera. Must match camera name on photon vision webpage.
        self.CARGOcam = photonvision._photonvision.PhotonCamera("CargoCam")
        #TODO: Verify GoalCam name on photon webpage.
        self.Goalcam = photonvision._photonvision.PhotonCamera("PiCam")

        # Switches to specified pipeline so starting pipeline is consistaint.
        # 2 is Blue Cargo, 1 is Red Cargo, and -1 is Driver Mode.
        self.CARGOcam.setPipelineIndex(1)

        self.CargoResults = self.CARGOcam.getLatestResult()
        self.CargoTarget = self.CargoResults.getBestTarget()

        self.GoalResults = self.Goalcam.getLatestResult()
        self.GoalTarget = self.GoalResults.getBestTarget()


    def CurrentCargoPipeline(self):
        return self.CARGOcam.getPipelineIndex()

    def getGoalX(self):
        return self.GoalTarget.getYaw()

    def getCargoX(self):
        return self.CargoTarget.getYaw()

    def getCargoY(self):
        return self.CargoTarget.getPitch()

    def setCargoRed(self):
        return self.CARGOcam.setPipelineIndex(1)

    def setCargoBlue(self):
        return self.CARGOcam.setPipelineIndex(2)

    def setDriverMode(self):
        return self.CARGOcam.setDriverMode(True)
