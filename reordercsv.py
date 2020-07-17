'''
reorders a csv file with openpose data. The first row (under the columns) must be an arbitrary row in which the 67th column is filled
'''


import rosbag, sys, csv
import time
import string
import os #for file management make directory
import shutil #for file management, copy file
import pandas as pd
import numpy as np
import math

#verify correct input arguments: 1 or 2
if (len(sys.argv) > 2 or len(sys.argv) <= 1):
    print "invalid number of arguments:   " + str(len(sys.argv))
    print "should be 2: 'reordercsv.py' and 'csvName'"
    sys.exit(1)

openpose_csv = sys.argv[1]
openpose_df = pd.read_csv(openpose_csv)

#order: lwrist, rwrist, neck, nose, lelbow, relbow, lshoulder, rshoulder, lhip, rhip, leye, reye, lear, rear

reorder_df = openpose_df.copy()

#Remove all unwanted columns
reorder_df = reorder_df.drop(reorder_df.columns[49:], axis=1)
reorder_df.columns = ['rosbagTimestamp', 'header', 'seq', 'stamp', 'secs', 'nsecs', 'frame_id',
        'lwrist.x', 'lwrist.y', 'lwrist.z', 'rwrist.x', 'rwrist.y', 'rwrist.z', 'neck.x', 'neck.y', 'neck.z', 'nose.x', 'nose.y', 'nose.z',
        'lelbow.x', 'lelbow.y', 'lelbow.z', 'relbow.x', 'relbow.y', 'relbow.z', 'lshoulder.x', 'lshoulder.y', 'lshoulder.z', 'rshoulder.x', 'rshoulder.y', 'rshoulder.z',
        'lhip.x', 'lhip.y', 'lhip.z', 'rhip.x', 'rhip.y', 'rhip.z', 'leye.x', 'leye.y', 'leye.z', 'reye.x', 'reye.y', 'reye.z', 'lear.x', 'lear.y', 'lear.z', 'rear.x', 'rear.y', 'rear.z']
#print(reorder_df.columns)
#dictionary for indexing
parts_dict = {'\"LWrist\"': 7, '\"RWrist\"': 10, '\"Neck\"': 13, '\"Nose\"': 16, '\"LElbow\"': 19, '\"RElbow\"' : 22, 
        '\"LShoulder\"' : 25, '\"RShoulder\"': 28, '\"LHip\"': 31, '\"RHip\"': 34, '\"LEye\"': 37, '\"REye\"': 40, '\"LEar\"': 43, '\"REar\"': 46}

#Remove first row & fill the dataframe with NaN where the column is a body part's coordinate
reorder_df = reorder_df.iloc[1:]
reorder_df.iloc[:, 7:] = np.nan

#print (openpose_df.iloc[:, 9])
#iterate through the open pose csv and fill the dataframe accordingly

rowNum = len(openpose_df)
colNum = len(openpose_df.columns)

for r in range(1,rowNum):
    for c in range(8,colNum,4):
        if(type(openpose_df.iloc[r,c]) is str):
            if(openpose_df.iloc[r,c] in parts_dict.keys()):
                i = parts_dict[openpose_df.iloc[r,c]]
                reorder_df.iloc[r-1,i] = openpose_df.iloc[r,c+1]
                reorder_df.iloc[r-1,i+1] = openpose_df.iloc[r,c+2]
                reorder_df.iloc[r-1,i+2] = openpose_df.iloc[r,c+3]
                #print(openpose_df.iloc[r,c])
                #print("i" + str(i))
                #print("row: " + str(r) + ", col: " + str(c))
                #print(str(reorder_df.iloc[r-1,i]) + " " + str(openpose_df.iloc[r,c+1]))
        else:
            break

#print(reorder_df["nose.x"])

reorder_df.to_csv("_slash_3d_reordered_data.csv", index=False)

print "Done"


