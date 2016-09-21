# SSM\_LinearArray
**Authors:** [Daobilige Su](https://sites.google.com/site/daobiligesu/home)

**Current version:** 1.0.0 

SSM\_LinearArray is a real-time SLAM library for sound sources mapping using an off-shelf robotic perception device (e.g. Kinect or PS3-Eye), which have a linear microphone array embedded inside. SSM\_LinearArray reads raw image (Mono/RGB-D), audio data and computes the camera trajectory, sound sources locations and a dense/sparse 3D reconstruction when a Kinect (**RGB-D** Camera) / PS3-Eye (**Monocular** Camera) is used.

###Video showing SSM\_LinearArray :
(Click on the image below to watch the youtube video or click [here](https://youtu.be/Ry_i3kmvlHM)) 
<a href="https://youtu.be/Ry_i3kmvlHM" target="_blank"><img src="https://img.youtube.com/vi/Ry_i3kmvlHM/0.jpg" 
alt="SSM_LinearArray Youtube Video" width="480" height="320" border="10" /></a>

<!--
#####Videos showing ORB-SLAM2:
<a href="http://www.youtube.com/watch?feature=player_embedded&v=dF7_I2Lin54
" target="_blank"><img src="http://img.youtube.com/vi/dF7_I2Lin54/0.jpg" 
alt="DataArena DataSet" width="240" height="180" border="10" /></a>


###Related Publications:

-->

#1. License

SSM\_LinearArray is released under a [GPLv3 license](https://github.com/daobilige-su/SSM_LinearArray/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/daobilige-su/SSM_LinearArray/blob/master/Dependencies.md).

For a closed-source version of SSM\_LinearArray for commercial purposes, please contact the authors: daobilige.su (at) student (dot) uts (dot) edu (dot) au.

<!--
If you use SSM\_LinearArray in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }
-->

#2. Prerequisites
I tested the library in **14.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
New thread and chrono functionalities of C++11 are used.

## Pangolin
[Pangolin](https://github.com/stevenlovegrove/Pangolin) is used for for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11**.

## Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are requiered by g2o (see below). On ubuntu:
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```

## DBoW2 and g2o (Included in Thirdparty folder)
Modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library are to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder. Original version of [libgp](https://github.com/mblum/libgp) is used for building microphone array sensor model using Gaussian Process.

## ROS
[ROS](ros.org) is needed to process the live input of the sensor or pre-recorded rosbag files.

## Freenect (optional)
[Freenect](https://github.com/OpenKinect/libfreenect) library is needed for live processing of kinect microphone array. In this case, once the freenect library is compiled, copy the libfreenect.so into *ROS/SSM_LinearArray/lib* folder. A precompiled libfreenect.so file under Ubuntu 14.04 is already included, so if it matches your version of OS, probably it's enough to go.

## PyAudio (optional)
[Pyaudio](https://pypi.python.org/pypi/PyAudio) library is needed for live processing of PS3-Eye microphone array. On Ubuntu, pyaudio can be installed by:
```
sudo apt-get install python-pyaudio
```

#3. Installation

Clone the repository:
```
git clone git@github.com:daobilige-su/SSM_LinearArray.git
```

A script `build.sh` is to build the *Thirdparty* libraries and *SSM\_LinearArray*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd SSM\_LinearArray
chmod +x build.sh
./build.sh
```

This will create **libSSM\_LinearArray.so**, **libcsparse_extension**  at *lib* folder and other executables in *ROS/SSM\_LinearArray/bin* folder.

#4. Run

## Running the pre-recorded data
In the case of Kinect, run the following commands in terminal:
```
roslaunch SSM_LinearArray freenectrosbag+ssmlineararray.launch 
```

In the case of PS3-Eye, run the following commands in terminal:
```
roslaunch SSM_LinearArray ps3eyerosbag+ssmlineararray.launch 
```

Open another terminal, go to the folder containing recorded rosbag files and play it by:
```
rosbag play XXX.bag
```
where XXX.bag is the recored rosbag file.

An example (recorded rosbag file) of mapping 2 sound sources using Kinect can be found [here](https://mega.nz/#!LpQHQAKY!ieHMVTvn84osptAr9ib6di18QmPL1oZ0KKhQdE_CNZg)(1GB).

## Running with live data
In the case of Kinect, the freenect firmware needed to be loaded to Kinect first. For details, have a look at instructions related audio data at [Freenect](https://github.com/OpenKinect/libfreenect) library. The firmware needed to loaded each time Kinect is reconnected to PC.
Then, run the following commands in terminal:
```
roslaunch SSM_LinearArray freenect+ssmlineararray.launch 
```

In the case of PS3-Eye, run the following commands in terminal:
```
roslaunch SSM_LinearArray ps3eye+ssmlineararray.launch 
```
  
#5. Processing your own sequences
Need to change the setting files with the calibration of your sensor. The setting files are inside the folder *ROS/SSM\_LinearArray/config*. The calibration model of OpenCV is used for camera calibration. 
