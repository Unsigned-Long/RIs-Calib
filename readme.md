# RIs-Calib: Multi-Radar Multi-IMU Spatiotemporal Calibrator

![Static Badge](https://img.shields.io/badge/Calibration-Multiple_Sensors-red) ![Static Badge](https://img.shields.io/badge/Cpp-17-green) ![ ](https://img.shields.io/badge/Radars-IMUs-blue) ![Static Badge](https://img.shields.io/badge/RIs-Calib-red) ![Static Badge](https://img.shields.io/badge/ROS-1.0-green) ![Static Badge](https://img.shields.io/badge/Python-3.0-blue) ![Static Badge](https://img.shields.io/badge/Continuous-Time-red) ![Static Badge](https://img.shields.io/badge/Bspline-Curves-green) ![Static Badge](https://img.shields.io/badge/Spatiotemporal-Calibrator-blue) ![Static Badge](https://img.shields.io/badge/WHU-SGG-red) ![Static Badge](https://img.shields.io/badge/ULong2-Shuolong_Chen-green) ![Static Badge](https://img.shields.io/badge/Wuhan-China-blue)

<div align=center><img src="./docs/img/ico.drawio.svg" width =100%></div>

## 0. Preliminaries

[![Typing SVG](https://readme-typing-svg.demolab.com?font=Ubuntu+Mono&weight=800&size=30&pause=1000&color=2DB845&background=2F90FF00&center=true&width=1000&lines=Thank+you+for+visiting!+I'm+ULong2%2C+always+here!)](https://git.io/typing-svg)

```cpp
+---------------+-------------------------------------------------+----------------------+
| Author(s)     | GitHub-Profile                                  | E-Mail               |
+---------------+-------------------------------------------------+----------------------+
| xxxxxxxxxxxxx | https://github.com/Unsigned-Long                | xxxxxxxxxxxxxxxxxx   |
+---------------+-------------------------------------------------+----------------------+
```

If you use ***RIs-Calib*** in a scientific publication, please cite the following  paper:smile::

```latex
# todo...
```

## 1. Overview

Aided inertial navigation system (INS), typically consisting of an inertial measurement unit (IMU) and an exteroceptive sensor, has been widely accepted and applied as a feasible solution for navigation. Compared with other aided INS, such as vision-aided INS and LiDAR-aided INS, radar-aided INS has better performance in adverse weather conditions such as fog and rain, due to the low-frequency signals radar utilizes. For such a radar-aided INS, accurate spatiotemporal transformation is a fundamental prerequisite to achieve optimal information fusion. In this paper, we present `RIs-Calib`: a spatiotemporal calibrator for multiple 3D radars and IMUs based on continuous-time estimation, which enables accurate spatial, temporal, and intrinsic calibration, and does not require any additional artificial infrastructure or prior knowledge. Our approach starts with a rigorous and robust procedure for state initialization, followed by batch optimizations, where all parameters could be refined to global optimal states steadily. We validate and evaluate `RIs-Calib` on both simulated and real-world experiments, and the results demonstrate that `RIs-Calib` is capable of accurate and consistent calibration. We open source our implementations at (**https://github.com/Unsigned-Long/RIs-Calib**) to benefit the research community.

<details open>
    <summary><b><i>Sensor suites & rotation and velocity B-splines in real-world experiments</i></b></summary>
    <div align=center>
        <img src="./docs/img/figures-real/rot-vel-splines.png" width =31%>
        <img src="./docs/img/figures-real/sensor-suites.png" width =30%>
        <img src="./docs/img/figures-real/splines.png" width =33%>
        </div>
</details>
<div align='center'><font size='5' color='red'><b><i>Click the Following Picture To See the Demo Video</i></b></font></div>

<div align=center>
<a href="https://youtu.be/4UI4JTBz4sk" title="RIs-Calib: An Open-source Spatiotemporal Calibrator for Multiple 3D Radars and IMUs"><img src="docs/img/video.jpg" alt="RIs-Calib: An Open-source Spatiotemporal Calibrator for Multiple 3D Radars and IMUs" width='100%'/></a>
</div>


## 2. Build RIs-Calib

### 2.1 Preparation

+ install `ROS` (for Ubuntu 20.04):

  ```bash
  sudo apt install ros-noetic-desktop-full
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

+ install `Ceres`:

  ```bash
  sudo apt-get install libceres-dev
  ```

+ install `Sophus`:

  see the GitHub Profile of **[Sophus](https://github.com/strasdat/Sophus.git)** library, clone it, compile it, and install it.

+ install `magic-enum`:

  see the GitHub Profile of **[magic-enum](https://github.com/Neargye/magic_enum.git)** library, clone it, compile it, and install it.

+ install `fmt`:

  ```bash
  sudo apt-get install libfmt-dev
  ```

+ install `Cereal`:

  ```bash
  sudo apt-get install libcereal-dev
  ```

+ install `spdlog`:

  ```bash
  sudo apt-get install libspdlog-dev
  ```

  

### 2.2 Clone and Compile RIs-Calib

+ clone RIs-Calib:

  ```bash
  git clone --recursive https://github.com/Unsigned-Long/RIs-Calib.git 
  ```

  change directory to '`{*}/RIs-Calib`', and run '`build_thirdparty.sh`'.
  
  ```bash
  cd {*}/RIs-Calib
  chmod +x build_thirdparty.sh
  ./build_thirdparty.sh
  ```
  
  this would build '`tiny-viewer`' and '`ctraj`' libraries.
  
+ prepare for thirdparty ros packages:

  + clone ros package '`ainstein_radar`' to '`{*}/RIs-Calib/src`' to support ros message '`ainstein_radar_msgs`':

    ```sh
    cd {*}/RIs-Calib/src
    git clone https://github.com/AinsteinAI/ainstein_radar.git
    ```

    then build this package:

    ```sh
    cd ..
    catkin_make -DCATKIN_WHITELIST_PACKAGES="ainstein_radar"
    ```

    Note that this package will depend on many other ros packages, you need to install them patiently.

  + clone ros package '`ti_mmwave_rospkg`' to '`{*}/RIs-Calib/src`' and than build this package:

    ```shell
    cd {*}/RIs-Calib/src
    git clone https://github.com/leicheng5/AWR1843_ROS.git
    cd ..
    catkin_make -DCATKIN_WHITELIST_PACKAGES="ti_mmwave_rospkg"
    ```

  + clone ros package '`sbg_driver`' to '`{*}/RIs-Calib/src`' and than build this package:

    ```shell
    cd {*}/RIs-Calib/src
    git clone https://github.com/SBG-Systems/sbg_ros_driver.git
    cd ..
    catkin_make -DCATKIN_WHITELIST_PACKAGES="sbg_driver"
    ```

+ change directory to the ros workspace (i.e., '`{*}/RIs-Calib`'), and run:

  ```bash
  cd {*}/RIs-Calib
  catkin_make -DCATKIN_WHITELIST_PACKAGES=""
  ```

  

## 3. Launch RIs-Calib

### 3.1 Simulation Test

We provide four simulation experiments, the corresponding data are in the '`{*}/RIs-Calib/src/ris_calib/output`' folder:

+ '`simu1`' and '`simu2`': one IMU and one radar.
+ '`simu3`' and '`simu4`' three IMUs and three radars.
+ Two scenes are simulated (eight-shape wave motion (left) and random motion (right)):

<div align=center><img src="./docs/img/traj1.png" width =50%><img src="./docs/img/traj2.png" width =50%></div>

To perform calibration for dataset '`simu1`' or '`simu2`', you should change field '`config_path`' in '`{*}/RIs-Calib/src/ris_calib/launch/ris-calib-prog.launch`' to:

```sh
$(find ris_calib)/config/config-simu12.yaml
```

The file '`config-simu12.yaml`' is a configure file for '`RIs-Calib`', which could be found in folder '`{*}/RIs-Calib/src/ris_calib/config`'. The detail configure information could determined be by yourself. Then, we launch '`RIs-Calib`':

```sh
roslaunch ris_calib ris-calib-prog.launch
```

The calibration results (left: splines for simu1, middle: splines for simu2, right: sensor suite):

<div align=center><img src="./docs/img/splines1.png" width =33%><img src="./docs/img/splines2.png" width =33%><img src="./docs/img/sensor12.png" width =33%></div>

For simu3 and simu4, you should change field '`config_path`' in '`{*}/RIs-Calib/src/ris_calib/launch/ris-calib-prog.launch`' to:

```sh
$(find ris_calib)/config/config-simu34.yaml
```

Then, we launch '`RIs-Calib`':

```sh
roslaunch ris_calib ris-calib-prog.launch
```

The calibration results (left: splines for simu3, middle: splines for simu4, right: sensor suite):

<div align=center><img src="./docs/img/splines3.png" width =33%><img src="./docs/img/splines4.png" width =33%><img src="./docs/img/sensor3.png" width =33%></div>

You could use scripts in '`{*}/RIs-Calib/src/ris_calib/scripts`' to draw figures:

<details>
    <summary><b><i>Distributions of IMU factors (accelerometer factor & gyroscope factor) in batch optimizations</i></b></summary>
    <div align=center><img src="./docs/img/figures/imu_residuals.png" width =100%><img src="./docs/img/figures/iter_info.png" width =100%></div>
</details>

<details>
    <summary><b><i>Convergence performance of spatiotemporal parameters for IMUs</i></b></summary>
    <div align=center><img src="./docs/img/figures/imu_simu_imu0.png" width =100%><img src="./docs/img/figures/imu_simu_imu1.png" width =100%><img src="./docs/img/figures/imu_simu_imu2.png" width =100%></div>
</details>

<details>
    <summary><b><i>Convergence performance of spatiotemporal parameters for Radars</i></b></summary>
    <div align=center><img src="./docs/img/figures/radar_simu_radar0.png" width =100%><img src="./docs/img/figures/radar_simu_radar1.png" width =100%><img src="./docs/img/figures/radar_simu_radar2.png" width =100%></div>
</details>

<details>
    <summary><b><i>Normal equations built during one iteration</i></b></summary>
    <div align=center>
        <b><i>Control points from rotation and velocity B-splines</i></b>
        <br/>
        <img src="./docs/img/lm_equ.png" width =80%>
        <br/>
        <b><i>All spatiotemporal parameters</i></b>
        <br/>
        <img src="./docs/img/batch_opt_lm_equ.png" width =70%></div>
</details>
<details>
    <summary><b><i>Rotation and velocity B-splines (left: simu3, right: simu4)</i></b></summary>
    <div align=center>
        <img src="./docs/img/simu3-splines.png" width =45%>
        <img src="./docs/img/simu4-splines.png" width =45%></div>
</details>

### 3.2 Real-world Experiments

The data of the real-world experiments we conducted are available here:

```latex
# Google Drive
link: https://drive.google.com/drive/folders/1_SPdmBnWIJTYyOIkyS0StbPMGVLdV_fw?usp=drive_link
```

or

```latex
# Baidu Netdisk
link: https://pan.baidu.com/s/1c6dNL6hdA_nx7K8Ud3nQvA
code: MRMI
```

Each data contains a ros bag, an information file, and a corresponding configuration file for solving:

+ `radars_imus.bag`: the ros bag which contains the measurements of two IMUs and radars, they are:

  ```latex
  path:        radars_imus.bag
  version:     2.0
  duration:    3:26s (206s)
  start:       Sep 26 2023 14:46:05.22 (1695710765.22)
  end:         Sep 26 2023 14:49:31.50 (1695710971.50)
  size:        69.7 MB
  messages:    347984
  compression: none [88/88 chunks]
  types:       sbg_driver/SbgImuData      [59cc541d794c367e71030fa700720826]
               sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
               ti_mmwave_rospkg/RadarScan [ca47afe7b19c0dbeb8f6b51574599509]
  topics:      /imu1/frame     82507 msgs    : sensor_msgs/Imu           
               /imu2/frame     40424 msgs    : sbg_driver/SbgImuData     
               /radar1/scan   130741 msgs    : ti_mmwave_rospkg/RadarScan
               /radar2/scan    94312 msgs    : ti_mmwave_rospkg/RadarScan
  ```

+ `duration.txt`: the file that records the time duration of the valid data piece (they are excited sufficiently, and thus could be used for calibration).
+ `config-real.yaml`: the corresponding configuration file.

The next steps are simple, just modify the file paths of the ros bag in the configuration file, and then configure the launch file of `RIs-Calib`, i.e., `ris-calib-prog.launch` in the `{*}/RIs-Calib/src/ris_calib` folder. Then, we launch '`RIs-Calib`':

```sh
roslaunch ris_calib ris-calib-prog.launch
```

You could use scripts in '`{*}/RIs-Calib/src/ris_calib/scripts`' to draw figures:

<details>
    <summary><b><i>Distributions of IMU factors (accelerometer factor & gyroscope factor) in batch optimizations</i></b></summary>
    <div align=center><img src="./docs/img/figures-real/imu_residuals.png" width =100%><img src="./docs/img/figures/iter_info.png" width =100%></div>
</details>

<details>
    <summary><b><i>Convergence performance of spatiotemporal parameters for IMUs</i></b></summary>
    <div align=center><img src="./docs/img/figures-real/imu_imu1_frame.png" width =100%><img src="./docs/img/figures-real/imu_imu2_frame.png" width =100%></div>
</details>

<details>
    <summary><b><i>Convergence performance of spatiotemporal parameters for Radars</i></b></summary>
    <div align=center><img src="./docs/img/figures-real/radar_radar1_scan.png" width =100%><img src="./docs/img/figures-real/radar_radar2_scan.png" width =100%></div>
</details>

<details open>
    <summary><b><i>Sensor suites & rotation and velocity B-splines</i></b></summary>
    <div align=center>
        <img src="./docs/img/figures-real/rot-vel-splines.png" width =31%>
        <img src="./docs/img/figures-real/sensor-suites.png" width =30%>
        <img src="./docs/img/figures-real/splines.png" width =33%>
        </div>
</details>
