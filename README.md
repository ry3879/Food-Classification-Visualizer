# Food-Classification-Visualizer

This visualizer is written in Python 2.7. The visualizer is intended to show how the model classified what is being shown in the video. There are two versions of the visualizer: fastVisualizer, which displays one person at a time, and fastVisualizer2, which displays two windows at once.

Features include:
- Pausing and Playing the visualizer: Upon pausing, graphs will automatically zoom in on the time you paused, with a 2 second margin. If the video is played while the graphs are zoomed in, the graphs will automatically update.
- Double clicking on the first graph will automatically zoom in on that location. The rest of the graphs and the video will also go to that location.
- A line is provided on the graphs for reference.
- There is a forwards and backwards button. Clicking it will go forwards 5 seconds and backwards 5 seconds respectively.
- A toolbar: this can revert the graph to the original graph (if zoomed in)

## How to get the necessary files
To use fastVisualizer.py, you must have an imu_data file, a video in avi format, and an openpose_data file (optional).

The visualizer script also requires a video. In order to produce a video from your bag file, you can use the following commands in preferably, an empty folder:

> rosrun image_view extract_images _sec_per_frame:=0.01 image:=/camera/color/image_raw/
> rosbag play *[BAG_FILE_NAME]*

You can use rosbag info (name) to check if you got the right number of frames, and if this doesn't work, you can try running the launch file in the image_view directory of ros.
Instructions for the launch file are here: http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

if the fps is 15 use:

> mencoder "mf://*.jpg" -fps 15 -o output.avi -ovc lavc -lavcopts vcodec=mjpeg

if the fps is 30 use:

> mencoder "mf://*.jpg" -fps 30 -o output.avi -ovc lavc -lavcopts vcodec=mjpeg

This command produces an mpg named output.avi. You will also have to have the library mencoder downloaded to do this.

To produce the openpose data file, you should have convert the rosbag's topic, 3d_data, to a csv. You can then use the command, **python reorder.py [openpose file]**, to reorder the csv. Then, you must run the reordered file with the command, **python metersToPixels.py [reordered file]** to convert the meters to pixels that can be plotted. This will then give you a file called openpose_pixels.csv, which we will use to run the visualizer. Note: the openpose data is only available to experts.

## How to run the visualizer (one window)

> python fastVisualizer.py [imu_data.csv] [video.avi] [openpose.csv]

Since novices do not have openpose data, you may leave out the openpose.csv file at the end. 

You should have the fastVisualizer, fastSegment, and animateOpenPose files in the same folder.

## How to run the visualizer (two windows)

> python fastVisualizer2.py [imu_data1.csv] [video1.avi] [openpose1.csv] [imu_data2.csv] [video2.csv] [openpose2.csv]

1 and 2 refer to the files of different experts/novices. Since novices do not have openpose data, in order to run this, replace openpose.csv with the word "novice" (without the quotes). This will automatically pop up 2 windows, where 1 is on the left and 2 is on the right. Unlike the visualizer with one window, this only has 2 graphs instead of 3.
