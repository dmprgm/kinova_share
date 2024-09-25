import numpy as np
import glob
import pandas as pd
from sklearn.cluster import AgglomerativeClustering
from scipy.spatial import procrustes


class Node:
    def __init__(self, path, data) -> None:
        self.path = path
        self.data = data

class HeirichialComparisons:

    def __init__(self) -> None:
        self.nodes = []
    
    def collectTrajectories(self):
        group = 0
        path = "/home/sharer/Documents/kinova_share/data_analysis/OUTPUTS/Trajectories/*.csv"
        for fname in glob.glob(path):
            data = pd.read_csv(fname)
            self.nodes.append(Node(path, data))
            group+=1
    
    def metricArray(self, x, y):
        
        x_1 = x.data['x'].to_numpy()
        x_2 = x.data['y'].to_numpy()
        x_3 = x.data['z'].to_numpy()
        length = len(y.data)
        xi = np.linspace(min(x), max(x), length)
        yi = np.linspace(min(y), max(y), length)


        #y.data['x','y','z'].to_numpy()
        procrustes(x,y)

        return 5

    def comparisons(self):
        #Setup number of groups equal to trajectories 
        #While groupings < 4 
            #For i in the set
                #Setup Max Value
                #For j in the set 
                    # Normalize Data To Same Lengths (Based on Longest Trajectory)
                    # If Max Value > Last replace it and store current reference
        pass



test = HeirichialComparisons()
test.collectTrajectories()
print(test.nodes)
test.metric(test.nodes[0],test.nodes[1])

