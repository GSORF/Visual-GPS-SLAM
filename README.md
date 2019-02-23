# Visual-GPS-SLAM
This is a repo for my master thesis research about the Fusion of Visual SLAM and GPS. It contains the thesis paper, code and other interesting data.

Note: Publication will take longer due to conference submissions and deadlines. The website that accompanies this research can be found at:
https://Master.Kalisz.co

# Master Thesis
The Master Thesis was written in LaTeX and will be added once any conferences related to it will be over.

# Publications
Two papers have been accepted and published related to this master thesis:

1. VISAPP 2019: "B-SLAM-SIM: A novel approach to evaluate the fusion of Visual SLAM and GPS by example of Direct Sparse Odometry and Blender" by Adam Kalisz, Florian Particke, Dominik Penk, Markus Hiller and Jörn Thielecke. Link (VISAPP technical program): http://insticc.org/node/TechnicalProgram/visigrapp/presentationDetails/73753
2. DICTA 2018: "Systematic Analysis of Direct Sparse Odometry" by Florian Particke, Adam Kalisz, Christian Hofmann, Markus Hiller, Henrik Bey and Jörn Thielecke. Link (IEEEXPLORE): https://ieeexplore.ieee.org/document/8615807



# Code
Code was written in C++ (main realtime implementation), Python (Blender Addon "B-SLAM-SIM" and Sensor data fusion in Blender), HTML5 (sensor recorder, track viewer, synchronization and live-demo tools).

## Dependencies
### Direct Sparse Odometry (DSO)
This work uses the Direct Sparse Odometry Project by TUM (see: https://vision.in.tum.de/dso and https://github.com/JakobEngel/dso). License: GPL-v3.

### Direct Sparse Odometry for ROS (ros-dso)
There is also a ros-wrapper for DSO, which is used for the real-time part of this work (see https://github.com/JakobEngel/dso_ros). The "catkin" branch by Nikolaus Demmel was actually used here, as the original "rosmake" version did not work for me. License: GPL-v3.
