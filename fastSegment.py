import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.collections import LineCollection
import cv2

#Function to read IMU data from a given CSV file
def read_file(file):
    imu_df = pd.read_csv(file)
    return imu_df

def get_gyro_df(imu_df,train=False): #Raw gyro data
    #select the columns you want for the three graphs in the order that you want them displayed & modify renaming accordingly
    #columns_sel=['secs','nsecs','z_1','y_2','z_2','Label']
    columns_sel = ['secs','nsecs', 'z', 'y.1', 'sum_force', 'Label']
    gyro_df = imu_df[columns_sel]
    
    #gyro_df=gyro_df.rename(index=str, columns={"z_1": "first","y_2": "second", "z_2":"third", 'secs':'Seconds','nsecs':'NanoSeconds', 'Label': 'label'})
    gyro_df=gyro_df.rename(index=str, columns={"z": "first","y.1": "second", "sum_force":"third", 'secs':'Seconds','nsecs':'NanoSeconds', 'Label': 'label'})
    r = len(imu_df)
    time = []
    
    #gyro_df['Seconds']=gyro_df['Seconds']-gyro_df['Seconds'][0] ##Seconds from 0
    starttime = gyro_df['Seconds'][0] + gyro_df['NanoSeconds'][0]/(10.0**9)
    for i in range(r):
        time.append(np.round(gyro_df['Seconds'][i]+gyro_df['NanoSeconds'][i]/(10.0**9),13))
    time = time - starttime
    gyro_df.insert(2, 'time', time, True)
    gyro_df.set_index(['Seconds','NanoSeconds'],drop=True,inplace=True)
    
    return gyro_df

def plot_gyro_df(self,gyro_df): 
    #if need to, can add more colors for more components
    colors = [tuple([0,0,1,1]), #blue 
            tuple([1,0,0,1]), #red
            tuple([1,1,0,1]), #yellow
            tuple([.5,0,.5,1]), #purple
            tuple([0,1,0,1]), #lime green
            tuple([0,1,1,1]), #cyan
            tuple([1,.5,0,1]), #orange
            tuple([1,.41,.71,1]), #pink
            tuple([0,.5,0,1]), #dark green
            tuple([.13,.70, .67,1])] #turquoise
    
    segmentsFirst = []
    segmentsSecond = []
    segmentsThird = []
    colorList = []
    
    #collect all line segments
    for i in range(len(gyro_df)-1) :
        segmentsFirst.append([(gyro_df['time'].iloc[i], gyro_df['first'].iloc[i]), (gyro_df['time'].iloc[i+1], gyro_df['first'].iloc[i+1])])
        segmentsSecond.append([(gyro_df['time'].iloc[i], gyro_df['second'].iloc[i]), (gyro_df['time'].iloc[i+1], gyro_df['second'].iloc[i+1])])
        segmentsThird.append([(gyro_df['time'].iloc[i], gyro_df['third'].iloc[i]), (gyro_df['time'].iloc[i+1], gyro_df['third'].iloc[i+1])])
        colorList.append(colors[gyro_df['label'].iloc[i]-1])
    
    #form a single line with LineCollection and refit the graph accordingly
    lastTime = gyro_df['time'].iloc[len(gyro_df)-1]
    self.ax.add_collection(LineCollection(segmentsFirst,colors=colorList, linewidths=2))
    self.ax.set_xlim(0, lastTime)
    self.ax.set_ylim(gyro_df['first'].min()-1, gyro_df['first'].max()+1)
    self.ax2.add_collection(LineCollection(segmentsSecond,colors=colorList, linewidths=2))
    self.ax2.set_ylim(gyro_df['second'].min()-1, gyro_df['second'].max()+1)
    self.ax2.set_xlim(0,lastTime)
    self.ax3.add_collection(LineCollection(segmentsThird,colors=colorList, linewidths=2))
    self.ax3.set_ylim(gyro_df['third'].min()-1, gyro_df['third'].max()+1)
    self.ax3.set_xlim(0,lastTime)

