# Food-Classification-Visualizer

To use fastVisualizer.py, you must have an imu_data file, a video, and an openpose_data file.

The visualizer script also requires a video. In order to produce a video from your bag file, you can use the following commands in preferably, an empty folder:

rosrun image_view extract_images _sec_per_frame:=0.01 image:=/camera/color/image_raw/
rosbag play [BAG_FILE_NAME]

You can use rosbag info (name) to check if you got the right number of frames, and if this doesn't work, you can try running the launch file in the image_view directory of ros.
Instructions for the launch file are here: http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

if the fps is 15 use:
mencoder "mf://*.jpg" -fps 15 -o output.avi -ovc lavc -lavcopts vcodec=mjpeg

if the fps is 30 use:
mencoder "mf://*.jpg" -fps 30 -o output.avi -ovc lavc -lavcopts vcodec=mjpeg

This command produced an mpg named output.avi. You will also have to have the library mencoder downloaded

To produce the openpose data file, you should have convert the rosbag's topic, 3d_data, to a csv. You can then use the command, python reorder.py [openpose file], to reorder the csv. Then, you must run the reordered file with the command, python metersToPixels.py [reordered file] to convert the meters to pixels that can be plotted.

So the command should look like
python fastVisualizer.py [imu_data.csv] [video.avi] [openpose.csv]

You should also have the fastVisualizer, fastSegment, and animateOpenPose files in the same folder since the visualizer file.

Features of the visualizer:
The video has the ability to pause and unpause. When you pause, the graphs automatically zoom in on the time you paused, with a 2 second margin. If you play while the graph is zoomed in, the graph will update along the time frame, staying at the the same zoomed in limits. The back button allows you to go 5 seconds back. The forward button causes you to go 5 seconds forward. You can also double click on the plot to go to that point in the video.

The toolbar has features like zooming in, returning to the original graph, etc ...
