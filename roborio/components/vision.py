# To adjust targeting settings within Photon Vision, go to 10.31.60.3:5800 in browser while on robot network.


import photonvision
from autonomous import #TODO: Import Alliance Color


class FROGVision:

    def __init__(self):

        # Sets varible to specifed camera. Must match camera name on photon vision web interface.
        self.CARGOcam = photonvision._photonvision.PhotonCamera("CargoCam")
        #TODO: Verify GoalCam name on photon web interface.
        self.Goalcam = photonvision._photonvision.PhotonCamera("PiCam")

        # Grabs the latest results from the Cameras and then get the best target.
        # Best target is chosen based off of the parameters on the photon vision web interface.
        self.CargoResults = self.CARGOcam.getLatestResult()
        self.CargoTarget = self.CargoResults.getBestTarget()

        self.GoalResults = self.Goalcam.getLatestResult()
        self.GoalTarget = self.GoalResults.getBestTarget()


    def setup(self):

        # Switches to specified pipeline depending on alliance color.
        # 2 is Blue Cargo, 1 is Red Cargo, and -1 is Driver Mode.
        #TODO: Modify if statement to get alliance color
        if "alliance color" == blue():
            self.CARGOcam.setPipelineIndex(2)

        elif "alliance color" == red():
            self.CARGOcam.setPipelineIndex(1)

        else:
            self.CARGOcam.setDriverMode(True)


    def CurrentCargoPipeline(self):
        return self.CARGOcam.getPipelineIndex()

    # Yaw is left- and right+ with center being 0
    def getGoalX(self):
        return self.GoalTarget.getYaw()

    def getCargoX(self):
        return self.CargoTarget.getYaw()

    # Pitch is up+ and down- with center being 0
    def getCargoY(self):
        return self.CargoTarget.getPitch()

    def setCargoRed(self):
        return self.CARGOcam.setPipelineIndex(1)

    def setCargoBlue(self):
        return self.CARGOcam.setPipelineIndex(2)

    def setDriverMode(self):
        return self.CARGOcam.setDriverMode(True)
