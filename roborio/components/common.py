def remap(val, OldMin, OldMax, NewMin, NewMax):
    """take a value in the old range and return a value in the new range"""
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
