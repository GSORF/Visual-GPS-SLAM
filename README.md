# Visual-GPS-SLAM
This is a repo for my master thesis research about the Fusion of Visual SLAM and GPS. It contains the thesis paper, code and other interesting data.

Note: Publication will take longer due to conference submissions and deadlines. The website that accompanies this research can be found at:
https://Master.Kalisz.co

# Master Thesis
The Master Thesis was written in LaTeX and will be added once any conferences related to it will be over.

# Conference Paper
An additional conference paper was recently (21.09.2018) accepted and is scheduled to be published at the end of 2018.

# Code
Code was written in C++ (main realtime implementation), Python (simulation), HTML5 (sensor recorder, track viewer, synchronization and live-demo tools).

## Dependencies
### Direct Sparse Odometry (DSO)
This work uses the Direct Sparse Odometry Project by TUM (see: https://vision.in.tum.de/dso and https://github.com/JakobEngel/dso). License: GPL-v3.

### Direct Sparse Odometry for ROS (ros-dso)
There is also a ros-wrapper for DSO, which is used for the real-time part of this work (see https://github.com/JakobEngel/dso_ros). The "catkin" branch by Nikolaus Demmel was actually used here, as the original "rosmake" version did not work for me. License: GPL-v3.
