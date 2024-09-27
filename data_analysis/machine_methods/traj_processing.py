import numpy as np
import glob
import pandas as pd
from sklearn.cluster import AgglomerativeClustering
from scipy.spatial import procrustes
from scipy.interpolate import interp1d
from scipy.stats import kurtosis, skew
import matplotlib.pyplot as plt

from collections import Counter

def count_integers(int_list):
    # Use Counter to count occurrences of each integer
    count = Counter(int_list)
    return count


class Node:
    def __init__(self, participant, path, data) -> None:
        self.participant = participant
        self.group = path
        self.data = data
        self.cluster=None

class TrajectoryProcessing:

    def __init__(self) -> None:
        self.nodes = []
    
    def collectTrajectories(self):
        group = 0
        path = '/home/sharer/Documents/kinova_share/data_analysis/OUTPUTS/Trajectories/*.csv'
        #path = "C:/Users/nnamd/Documents/GitHub/kinova_share/data_analysis/OUTPUTS/Trajectories/*.csv"
        for fname in glob.glob(path):
            name = fname[80:]
            name = name.split('.')[0]
            list = name.split('_')
            print(list[1], list[2][0])
            data = pd.read_csv(fname)
            self.nodes.append(Node(list[1][1], list[2][0], data))
            group+=1
    

    def velocityCalc(self):
        distances = ['x','y','z']
        for i in range(len(self.nodes)):
            positions = self.nodes[i].data[distances].to_numpy()
            time = self.nodes[i].data['time'].to_numpy()
            velocity = np.gradient(positions,time,axis=0)
            acceleration = np.gradient(velocity, time, axis=0)
            self.nodes[i].data[['x_vel','y_vel','z_vel']] = velocity
            self.nodes[i].data[['x_accel','y_accel','z_accel']] = acceleration

    def kurtosisCalc(self):
        self.k_values = []
        for i in range(len(self.nodes)):
            df = self.nodes[i].data
            k_values = kurtosis(df.loc[:, df.columns != 'time'])
            self.k_values.append(np.array(k_values))
            #self.k_values[['x','y','z','x_vel','y_vel','z_vel','x_accel','y_accel','z_accel']] = k_values
        self.k_values = np.array(self.k_values)
        print(np.mean(self.k_values, axis=0))
        

    def skewnessCalc(self):
        self.skewness = []
        for i in range(len(self.nodes)):
            df = self.nodes[i].data
            skewness = skew(df.loc[:, df.columns != 'time'])
            self.skewness.append(skewness)
            #self.k_values[['x','y','z','x_vel','y_vel','z_vel','x_accel','y_accel','z_accel']] = k_values
        self.skewness = np.array(self.skewness)
        print(np.mean(self.skewness, axis=0))


    def plotSkewKurt(self):
        #for i in range(len(self.nodes)):
        print(self.k_values)
        y = self.k_values[:,0]
        x = self.skewness[:,0]
        #plt.scatter(x,y)

        y = self.k_values[:,1]
        x = self.skewness[:,1]
        #plt.scatter(x,y,c='red')

        y = self.k_values[:,2]
        x = self.skewness[:,2]
        plt.scatter(x,y,c='green')


        y = self.k_values[:,3]
        x = self.skewness[:,3]
        plt.scatter(x,y,c='purple')


        y = self.k_values[:,4]
        x = self.skewness[:,4]
        plt.scatter(x,y,c='orange')

        y = self.k_values[:,5]
        x = self.skewness[:,5]
        plt.scatter(x,y,c='yellow')

        plt.show()


        

    def metric(self, x, y):
        array_x = x.data[['x','y','z']].to_numpy()
        array_y = y.data[['x','y','z']].to_numpy()
        if array_x.shape[0] > array_y.shape[0]:
            larger = x 
            smaller = y 
        else:
            larger = y 
            smaller = x

        larger_array = larger.data[['x','y','z']].to_numpy()
        smaller_array = smaller.data[['x','y','z']].to_numpy()
        smaller_time = smaller.data['time'].to_numpy()
        length = len(larger.data['time'].to_numpy())

        time_new = np.linspace(smaller_time.min(), smaller_time.max(), length)
        f = interp1d(smaller_time, smaller_array, axis=0)
        
        corrected_array = f(time_new)

        mtx1, mtx2, disparity = procrustes(corrected_array, larger_array)
        return disparity
    
    def plotConditions(self, conditions):
        fig = plt.figure()
        ax = plt.axes(projection ='3d')
        for x in self.nodes:
            if x.group in conditions:
                x_d = x.data['x'].to_numpy()
                y_d = x.data['y'].to_numpy()
                z_d = x.data['z'].to_numpy()
                if x.group==conditions[0]:
                    ax.plot3D(x_d,y_d,z_d,'green')
                else:
                    ax.plot3D(x_d,y_d,z_d,'blue')
        plt.show()

    def distanceMatrix(self):
        results = []
        print(len(self.nodes))
        for i in range(len(self.nodes)):
            print(i)
            temp = []
            for j in range(len(self.nodes)):
                temp.append(self.metric(self.nodes[i],self.nodes[j]))
            results.append(temp)
        self.results = results
        return np.array(results)

    def groupBasedOnCluster(self):
        distance = self.distanceMatrix()
        cluster = AgglomerativeClustering(8, metric='precomputed', linkage='complete')
        cluster.fit(distance)
        for i in range(len(cluster.labels_)):
            self.nodes[i].cluster = cluster.labels_[i]
        

    def plotClusters(self, selection=[0,1]):
        fig = plt.figure()
        ax = plt.axes(projection ='3d')
        colors = ['green', 'blue', 'red', 'yellow', 'purple', 'black', 'orange','pink']
        for x in self.nodes:
            if x.cluster in selection:
                x_d = x.data['x'].to_numpy()
                y_d = x.data['y'].to_numpy()
                z_d = x.data['z'].to_numpy()
                ax.plot3D(x_d,y_d,z_d,colors[x.cluster])
        plt.show()



test = TrajectoryProcessing()
test.collectTrajectories()
test.velocityCalc()
test.kurtosisCalc()
test.skewnessCalc()
test.plotSkewKurt()
#test.plotConditions(['G','H'])
#test.groupBasedOnCluster()
#test.plotClusters()
#print(test.nodes)
# print(test.metric(test.nodes[0],test.nodes[1]))
# 
# print(count_integers(cluster.labels_))

# import matplotlib.pyplot as plt

# fig = plt.figure()
# ax = plt.axes(projection ='3d')
# ax.plot3D(test.nodes[0].data['x'].to_numpy(), test.nodes[0].data['y'].to_numpy(), test.nodes[0].data['z'].to_numpy())
# ax.plot3D(test.nodes[1].data['x'].to_numpy(), test.nodes[1].data['y'].to_numpy(), test.nodes[1].data['z'].to_numpy())
# ax.plot3D(test.nodes[2].data['x'].to_numpy(), test.nodes[2].data['y'].to_numpy(), test.nodes[2].data['z'].to_numpy())
# plt.show()