# SD-SLAM
**Authors:** [Eduardo Perdices](https://gsyc.urjc.es/~eperdices/)

SD-SLAM is a real-time SLAM library for **Monocular** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time.

# 1. License

SD-SLAM is released under a [GPLv3 license](https://github.com/eperdices/SD-SLAM/blob/master/License-gpl.txt).

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04** and **Ubuntu 18.04**, but it should be easy to compile in other platforms.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
We use [Eigen3](http://eigen.tuxfamily.org) to perform matrices operations. Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## g2o (Included in extra folder)
We use modified versions of the [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. This library (which is BSD) is included in the *extra* folder.

# 3. Building ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/eperdices/SD-SLAM
```

We provide a script `build.sh` to build *SD-SLAM*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd SD-SLAM
chmod +x build.sh
./build.sh
```

This will create **libSD_SLAM.so**  at *lib* folder and the executables **monocular** and **rgbd** in *Examples* folder.

# 4. Monocular Examples

Inside `PATH_TO_SEQUENCE_FOLDER` there must be a file named `files.txt` with each image filename.

```
./Examples/Monocular/monocular Examples/Monocular/X.yaml PATH_TO_SEQUENCE_FOLDER
```

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Change `X.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively.

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Change `X.yaml` to EuRoC.yaml.

## Live Camera

You can use a USB camera to run SD-SLAM with live images. Change `N` to the proper `/dev/videoN` value where your camera is connected.

```
./Examples/Monocular/monocular Examples/Monocular/X.yaml N
```

## Initialization with pattern

A pattern can be emploid to create the initial map using real scale. The pattern detected is the same chessboard used for camera calibration, located inside `Examples/Calibration` folder.

To use a pattern, set `UsePattern: 1` in yaml configuration file.

# 5. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script `associate.py` located inside `Examples/RGB-D` folder and executing:

  ```
  python Examples/RGB-D/associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 6. Monocular and IMU Fusion Example

Inside `PATH_TO_SEQUENCE_FOLDER` there must be a file named ''files.txt'' with each image filename. IMU data has to be stored in a csv file where each line has a timestamp and 6 measurements (3 from gyroscope and 3 from accelerometer).

  ```
  ./Examples/Fusion/monocular_imu Examples/Monocular/X.yaml PATH_TO_SEQUENCE_FOLDER IMU_DATA_FILE.csv
  ```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Change `X.yaml` to EuRoC.yaml.

# 7. Processing your own sequences

You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM dataset for monocular or RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the SD-SLAM library and how to pass images to the SLAM system. RGB-D input must be synchronized and depth registered.

## Camera Calibration

We provide a tool to calibrate your camera using the calibration model of OpenCV. You need to print the pattern found inside ` Examples/Calibration` folder and take several pictures of this pattern from different points of view. Then store these pictures in a folder and execute the calibration tool.

  ```
  ./Examples/Calibration/calibration PATH_TO_IMAGES_FOLDER
  ```

Inside `PATH_TO_IMAGES_FOLDER` there must be a file named ''files.txt'' with each image filename.

You can check if the intrinsic parameters calculated are accurate checking the rectified images stored in `PATH_TO_IMAGES_FOLDER`.

# 8. ROS Examples

### Building the node
1. Add the path including *Examples/ROS/SD-SLAM* to the ROS_PACKAGE_PATH environment variable. Replace PATH by the folder where you cloned SD-SLAM:

  ```
  echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:PATH/SD-SLAM/Examples/ROS" >> ~/.bashrc
  ```

2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```

### Running Monocular Node

Monocular node reads RGB images from topic `/camera/rgb/image_raw`. You will need to provide a settings file (see examples above).

  ```
  rosrun SD-SLAM Monocular Examples/ROS/SD-SLAM/ROS.yaml
  ```

### Running RGBD Node

RGBD node reads RGB images from topic `/camera/rgb/image_raw` and depth images from topic `/camera/depth/image_raw`. You will need to provide a settings file (see examples above).

  ```
  rosrun SD-SLAM RGBD Examples/ROS/SD-SLAM/ROS.yaml
  ```

### Running Fusion Node

Fusion node reads RGB images from topic `/camera/rgb/image_raw` and IMU data from topic `/imu_data`. You will need to provide a settings file (see examples above).

  ```
  rosrun SD-SLAM Fusion Examples/ROS/SD-SLAM/ROS.yaml
  ```

# 9. Reading saved data

Mapping data can be stored in a YAML file. You can save the current map at any moment pressing the `Stop and Save` button, or it will be created automatically when a sequence is completed.

Saved data can be loaded afterwards both in monocular and RGBD modes:

## 9.1 Monocular Example

Add a new parameter at the end of the standard monocular command. Change `PATH_TO_SAVED_MAP` to the path to the corresponding YAML file.

```
./Examples/Monocular/monocular Examples/Monocular/X.yaml PATH_TO_SEQUENCE_FOLDER PATH_TO_SAVED_MAP
```

## 9.2 RGBD Example

Add a new parameter at the end of the standard RGBD command. Change `PATH_TO_SAVED_MAP` to the path to the corresponding YAML file.

```
./Examples/RGB-D/rgbd Examples/RGB-D/X.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE PATH_TO_SAVED_MAP
```

## 9.3 ROS Monocular Node Example

Add a new parameter at the end of the standard ROS monocular command. Change `PATH_TO_SAVED_MAP` to the path to the corresponding YAML file.

```
rosrun SD-SLAM Monocular Examples/ROS/SD-SLAM/ROS.yaml PATH_TO_SAVED_MAP
```

## 9.4 ROS RGBD Node Example

Add a new parameter at the end of the standard ROS RGBD command. Change `PATH_TO_SAVED_MAP` to the path to the corresponding YAML file.

```
rosrun SD-SLAM RGBD Examples/ROS/SD-SLAM/ROS.yaml PATH_TO_SAVED_MAP
```

# 10. Localization Mode

You can change between the *Default Mode* and *Localization mode* using the GUI of the map viewer.

### Default Mode
The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds a new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed.

# 11. Android Compilation

You can create a SD-SLAM library for Android running:

  ```
  chmod +x build_android.sh
  ./build_android.sh
  ```

Before running this script, read https://github.com/taka-no-me/android-cmake to configure your workspace properly.
