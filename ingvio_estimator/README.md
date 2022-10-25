# InGVIO

An invariant filter for visual-inertial-raw GNSS fusion.

**Paper:** InGVIO: A Consistent Invariant Filter For Fast and High-Accuracy GNSS-Visual-Inertial Odometry

**Paper Author:** Changwu Liu, Chen Jiang and Haowen Wang

**Paper Status:** Manuscript submitted to IEEE RA-L for possible publication. Preprint version available on ArXiv.

**Current Paper Link:** The pre-print version is available on ArXiv.

InGVIO is an invariant filter approach for fusion of monocular/stereo camera, IMU and raw GNSS measurements including pseudo ranges and Doppler shifts. InGVIO is intrinsically consistent under conditional infinitesimal invariance of the GNSS-Visual-Inertial system. InGVIO has the following key features: (a) fast due to decoupled IMU propagation, key-frame marginalization strategy and no SLAM-features; (b) accurate due to intrinsic consistency maintenance; (c) better convergence properties than 'naive' EKF-based filters.

Moreover, we offer our fixed-wing datasets in the form of ROS Bags including stereo-visual, IMU and raw-GNSS measurements.

**Fixed-Wing Dataset Link:** Coming Soon.

The links to the datasets will be continuously updated. The config files for this dataset are contained in the InGVIO code configuration in path 'config/fw_zed2i_f9p'.

## 1. System Requirements

### 1.1  Support of C++ 14 Features

The compiler should at least support c++14 standards.

### 1.2  ROS-Noetic System

InGVIO is developed under [ROS-Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) with its default OpenCV4 library. However, InGVIO should be working on ROS-Melodic with OpenCV3. In the future, we may add support to ROS 2.

### 1.3  Eigen Library

Eigen is a fantastic matrix computation library. InGVIO is developed under [Eigen3.3.7](https://eigen.tuxfamily.org/index.php?title=Main_Page). Other Eigen 3 versions should be OK for InGVIO.

### 1.4  SuiteSparse Library

We use [SuiteSparse](https://github.com/DrTimothyAldenDavis/SuiteSparse/releases) Library for sparse QR-decomposition in visual updates. 

### 1.5  gnss_comm Library

A wrapper for GNSS messages in ROS. See [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm). The fantastic optimization-based work [GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS) also relies on this library. We reserve a copy of gnss_comm in this repo.

## 2. Build InGVIO

Download or clone this repo to your ROS workspace.

```
cd ~/ws_catkin
catkin_make
```

Source the setup file to let ROS recognize the related launch files.

```
source devel/setup.bash
```

## 3. Run with Our Fixed-Wing Datasets

First download and unzip our fixed-wing dataset to your environment.

The 'fw_zed2i_f9p' folder contains the configuration files for fixed-wing datasets. To run InGVIO in your environment, the following should be conducted first.

Please adjust and modify the paths in 'config/fw_zed2i_f9p/ingvio_mono.yaml' and 'config/fw_zed2i_f9p/ingvio_stereo.yaml' to the case in your environment.

If you want to record the output topics, please set your output dir in 'ingvio/launch/ingvio_mono, stereo_fw_record.launch' files.

To run with monocular camera, please enter:

```
roslaunch ingvio_estimator ingvio_mono_fw.launch
```

For stereo-case, please enter:

```
roslaunch ingvio_estimator ingvio_stereo_fw.launch
```

To record the output topics, you can directly use our script file by:

```
roslaunch ingvio_estimator ingvio_mono_fw_record.launch
roslaunch ingvio_estimator ingvio_stereo_fw_record.launch
```

Rviz will be automatically launched. Run the ROS Bag to see the results:

```
rosbag play fw_easy.bag --pause
```

## 4. Run with GVINS Public Datasets

### 4.1 Configuration Settings

The GVINS datasets can be acquired from [GitHub - HKUST-Aerial-Robotics/GVINS-Dataset](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset). The config files for GVINS Datasets are already integrated in 'config/sportsfield'. It's valid for all 3 datasets provided by GVINS.

To run InGVIO in your environment, the following should be conducted first.

Please adjust and modify the paths in 'config/sports_field/ingvio_mono.yaml' and 'config/sports_field/ingvio_stereo.yaml' to the case in your environment.

If you want to record the output topics, please set your output dir in 'ingvio/launch/ingvio_mono, stereo_sf_record.launch' files.

To run with monocular camera, please enter:

```
roslaunch ingvio_estimator ingvio_mono_sf.launch
```

For stereo-case, please enter:

```
roslaunch ingvio_estimator ingvio_stereo_sf.launch
```

To record the output topics, you can directly use our script file by:

```
roslaunch ingvio_estimator ingvio_mono_fw_record.launch
roslaunch ingvio_estimator ingvio_stereo_fw_record.launch
```

Rviz will be automatically launched. Run the ROS Bag to see the results:

```
rosbag play sports_field.bag --pause
rosbag play indoors_outdoors.bag --pause
rosbag play urban_driving.bag --pause
```

### 4.2 Parameter Tuning Hints

**For sports_field.bag:** Monocular/Stereo Case: max_pts_frame = 150/110, visual_noise = 0.12, gnss_chi2_test = false, imu_buffer_size = 3000.

**For urban_driving.bag:** Monocular/Stereo Case: max_pts_frame = 150/110, visual_noise = 0.12, gnss_chi2_test = true, imu_buffer_size = 3000. To avoid polluted GNSS measurements, a stronger version could be gnss_chi2_test = false, gnss_strong_reject = true.

**For indoors_outdoors.bag:** Monocular/Stereo Case: max_pts_frame = 150/110, visual_noise = 0.18, gnss_chi2_test = false, gnss_strong_reject = true. 

Please modify or try other parameters if the above behaves not good. Good parameters may be different under different environment settings.

## 5. Acknowledgements

The realization of type-based index system in filter framework is inspired by [OpenVINS](https://github.com/rpng/open_vins). The author himself has learned lots of programming skills in SLAM by reading the codes of OpenVINS.

The [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) provided by GVINS is a great wrapper in developing codes involving raw GNSS measurements in ROS.

## 6. License

This software is open-sourced under GPLv3 license. See [GNU GPL v3.0 - GNU](https://www.gnu.org/licenses/gpl-3.0.html).

Please cite our paper if you use either our code or our fixed-wing datasets.
