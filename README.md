# Visual-GPS-SLAM
This is a repo for my master thesis research about the Fusion of Visual SLAM and GPS. It contains the thesis paper, code and other interesting data.

The website that accompanies this research can be found at:
https://Master.Kalisz.co

# Master Thesis
The Master Thesis was written in LaTeX and is published here:
http://master.kalisz.co/MasterThesis_AdamKalisz_Online.pdf

# Video
This video shows first tests with the setup running in a car:
[![Watch the video]()](https://master.kalisz.co/video/Robolab_Demo_Kalisz_MusicSTEEP.mp4)



# Publications
Two papers have been accepted and published related to this master thesis:

1. VISAPP 2019: "B-SLAM-SIM: A novel approach to evaluate the fusion of Visual SLAM and GPS by example of Direct Sparse Odometry and Blender" by Adam Kalisz, Florian Particke, Dominik Penk, Markus Hiller and Jörn Thielecke. Link (VISAPP technical program): http://insticc.org/node/TechnicalProgram/visigrapp/presentationDetails/73753
2. DICTA 2018: "Systematic Analysis of Direct Sparse Odometry" by Florian Particke, Adam Kalisz, Christian Hofmann, Markus Hiller, Henrik Bey and Jörn Thielecke. Link (IEEEXPLORE): https://ieeexplore.ieee.org/document/8615807



# Code
Code was written in C++ (main realtime implementation), Python (Blender Addon "B-SLAM-SIM" and Sensor data fusion in Blender), HTML5 (sensor recorder, track viewer, synchronization and live-demo tools).

## How to recreate the Plots from the thesis
In order to recreate the plots from the thesis and papers you can use the attached python script. First, make sure you are using Python3 and have "matplotlib" installed. For the latter you can usually do:
```bash
pip install matplotlib
```

This repository contains a larger amount of .txt files which have been generated in the sensor data fusion step within Blender. The location of those files is "02_Utilities/FusionLinearKalmanFilter" (the sensor data fusion script and a .blend file are located there as well!). These .txt files can be converted to plots using the following command:
```python
python BlenderPlot.py
```
If you want to test your own sensor data fusion, you can use the Blender 2.79 (no 2.8 at the moment) "01_LinearKalmanFilter_allEvaluations.blend" file in "02_Utilities/FusionLinearKalmanFilter" and take a look at how sensor data fusion is done there. Please, if you have any questions related to that, feel free to contact me or just open an issue. I will respond as soon as possible! I have planned to create a video tutorial on how to use it, but didn't manage to finish it yet. This is still on my TODO list and will most likely make everything a lot clearer.

## Why GPLv3 license?
I have modified code that is licensed under GNU GPL Version 3 (see below), so I have to make my changes available under the same license. I can provide a different license for my custom Blender Addon scripts if you prefer that. Just feel free to message me.

## Dependencies
### Direct Sparse Odometry (DSO)
This work uses the Direct Sparse Odometry Project by TUM (see: https://vision.in.tum.de/dso and https://github.com/JakobEngel/dso). License: GPL-v3.

### Direct Sparse Odometry for ROS (ros-dso)
There is also a ros-wrapper for DSO, which is used for the real-time part of this work (see https://github.com/JakobEngel/dso_ros). The "catkin" branch by Nikolaus Demmel was actually used here, as the original "rosmake" version did not work for me. License: GPL-v3.
