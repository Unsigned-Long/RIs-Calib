# RIs-Calib: Multi-Radar Multi-IMU Spatiotemporal Calibrator

![Static Badge](https://img.shields.io/badge/Calibration-Multiple_Sensors-red) ![Static Badge](https://img.shields.io/badge/Cpp-17-green) ![ ](https://img.shields.io/badge/Radars-IMUs-blue) ![Static Badge](https://img.shields.io/badge/RIs-Calib-red) ![Static Badge](https://img.shields.io/badge/ROS-1.0-green) ![Static Badge](https://img.shields.io/badge/Python-3.0-blue) ![Static Badge](https://img.shields.io/badge/Continuous-Time-red) ![Static Badge](https://img.shields.io/badge/Bspline-Curves-green) ![Static Badge](https://img.shields.io/badge/Spatiotemporal-Calibrator-blue) ![Static Badge](https://img.shields.io/badge/WHU-SGG-red) ![Static Badge](https://img.shields.io/badge/ULong2-Shuolong_Chen-green) ![Static Badge](https://img.shields.io/badge/Wuhan-China-blue)

<div align=center><img src="docs/img/ico2.drawio.svg" width =100%></div>


### 3.2 Real-world Experiments

The data of the real-world experiments we conducted are available here:

```latex
# Google Drive
https://drive.google.com/drive/folders/1_SPdmBnWIJTYyOIkyS0StbPMGVLdV_fw?usp=drive_link
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

The next steps are simple, just modify the file paths of the ros bag in the configuration file, and then configure the launch file of `RIs-Calib`, i.e., `ris-calib-prog.launch` in the `ris_calib` folder. Then, we launch '`RIs-Calib`':

```sh
roslaunch ris_calib ris-calib-prog.launch
```

You could use scripts in '`ris_calib/scripts`' to draw figures:

<details open>
    <summary><b><i>Distributions of IMU factors (accelerometer factor & gyroscope factor) in batch optimizations</i></b></summary>
    <div align=center><img src="docs/img/figures-real/imu_residuals.png" width =80%><img src="docs/img/figures/iter_info.png" width =80%></div>
</details>

<details open>
    <summary><b><i>Convergence performance of spatiotemporal parameters for IMUs</i></b></summary>
    <div align=center><img src="docs/img/figures-real/imu_imu1_frame.png" width =80%><img src="docs/img/figures-real/imu_imu2_frame.png" width =80%></div>
</details>

<details open>
    <summary><b><i>Convergence performance of spatiotemporal parameters for Radars</i></b></summary>
    <div align=center><img src="docs/img/figures-real/radar_radar1_scan.png" width =80%><img src="docs/img/figures-real/radar_radar2_scan.png" width =80%></div>
</details>


<details open>
    <summary><b><i>Sensor suites & rotation and velocity B-splines</i></b></summary>
    <div align=center>
        <img src="docs/img/figures-real/rot-vel-splines.png" width =31%>
        <img src="docs/img/figures-real/sensor-suites.png" width =30%>
        <img src="docs/img/figures-real/splines.png" width =33%>
        </div>
</details>
