import numpy
import glob

class HeirichialComparisons:

    def __init__(self) -> None:
        pass

    
    def collectTrajectories(self):
        
        path = "C:/Users/nnamd/Documents/GitHub/kinova_share/data_analysis/OUTPUTS/Trajectories/*.csv"
        for fname in glob.glob(path):
            print(fname)

HeirichialComparisons().collectTrajectories()

