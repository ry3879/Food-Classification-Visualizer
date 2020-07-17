import sys, csv
import time
import string
import os #for file management make directory
import shutil #for file management, copy file
import pandas as pd
import numpy as np
import math

proj = np.array([[616.7794799804688, 0.0, 317.8077697753906],
            [0.0, 616.8223876953125, 238.88868713378906], 
            [0.0, 0.0, 1.0]])

#verify correct input arguments: should just have 2
if (len(sys.argv) > 2 or len(sys.argv) <= 1):
    print "invalid number of arguments:   " + str(len(sys.argv))
    print "should be 2: 'metersToPixel.py' and 'csvName'"
    sys.exit(1)

openpose_csv = sys.argv[1]
openpose_df = pd.read_csv(openpose_csv)

#order: lwrist, rwrist, neck, nose, lelbow, relbow, lshoulder, rshoulder, lhip, rhip, leye, reye, lear, rear

#grab the columns
columns_sel = ['rosbagTimestamp', 'header', 'seq', 'stamp', 'secs', 'nsecs', 'frame_id',
        'lwrist.x', 'lwrist.y','rwrist.x', 'rwrist.y','neck.x', 'neck.y','nose.x', 'nose.y',
        'lelbow.x', 'lelbow.y','relbow.x', 'relbow.y','lshoulder.x', 'lshoulder.y', 'rshoulder.x', 'rshoulder.y',
        'lhip.x', 'lhip.y', 'rhip.x', 'rhip.y','leye.x', 'leye.y', 'reye.x', 'reye.y', 'lear.x', 'lear.y', 'rear.x', 'rear.y']

pixels_df = openpose_df.copy()
#print(len(pixels_df))
pixels_df = pixels_df.drop(pixels_df.columns[35:], axis=1)
pixels_df.columns = columns_sel
length = len(openpose_df)
col_length = len(openpose_df.columns)

r = 7
for i in range (7, col_length, 3):
    name1 = columns_sel[r]
    name2 = columns_sel[r+1]
    for j in range(0, length):
        if(not math.isnan(openpose_df.iloc[j][i])):
            x = openpose_df.iloc[j][i+1]
            y = openpose_df.iloc[j][i+2]
            depth = openpose_df.iloc[j][i]
            if(depth != 0):
                x = x/depth
                y= y/depth
            coord = np.array([[x],[y],[1]])
            result = np.matmul(proj,coord)
            #print(result)
            #print(result[0][0])
            #pixels_df[name1][j] = round(result[0][0])
            #pixels_df[name2][j] = round(result[1][0])
            pixels_df.iat[j,r] = round(result[0][0])
            pixels_df.iat[j,r+1] = round(result[1][0])
        else:
            pixels_df.iat[j,r] = float("nan")
            pixels_df.iat[j,r+1] = float("nan")

    r+=2

pixels_df.to_csv("openpose_pixels.csv", index=False)


            
