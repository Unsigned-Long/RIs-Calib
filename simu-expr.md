# RIs-Calib: Multi-Radar Multi-IMU Spatiotemporal Calibrator

![Static Badge](https://img.shields.io/badge/Calibration-Multiple_Sensors-red) ![Static Badge](https://img.shields.io/badge/Cpp-17-green) ![ ](https://img.shields.io/badge/Radars-IMUs-blue) ![Static Badge](https://img.shields.io/badge/RIs-Calib-red) ![Static Badge](https://img.shields.io/badge/ROS-1.0-green) ![Static Badge](https://img.shields.io/badge/Python-3.0-blue) ![Static Badge](https://img.shields.io/badge/Continuous-Time-red) ![Static Badge](https://img.shields.io/badge/Bspline-Curves-green) ![Static Badge](https://img.shields.io/badge/Spatiotemporal-Calibrator-blue) ![Static Badge](https://img.shields.io/badge/WHU-SGG-red) ![Static Badge](https://img.shields.io/badge/ULong2-Shuolong_Chen-green) ![Static Badge](https://img.shields.io/badge/Wuhan-China-blue)

<div align=center><img src="docs/img/ico2.drawio.svg" width =100%></div>

### 3.1 Simulation Test

We provide four simulation experiments, the corresponding data are in the '`ris_calib/output`' folder:

+ '`simu1`' and '`simu2`': one IMU and one radar.
+ '`simu3`' and '`simu4`' three IMUs and three radars.
+ Two scenes are simulated (eight-shape wave motion (left) and random motion (right)):

<div align=center><img src="docs/img/traj1.png" width =50%><img src="docs/img/traj2.png" width =50%></div>

To perform calibration for dataset '`simu1`' or '`simu2`', you should change field '`config_path`' in '`ris_calib/launch/ris-calib-prog.launch`' to:

```sh
$(find ris_calib)/config/config-simu12.yaml
```

The file '`config-simu12.yaml`' is a configure file for '`RIs-Calib`', which could be found in folder '`/ris_calib/config`'. The detail configure information could determined be by yourself. Then, we launch '`RIs-Calib`' (**before launch `RIs-Calib`, remember change the path or directory in the yaml-format configure file!**):

```sh
roslaunch ris_calib ris-calib-prog.launch
```

The calibration results (left: splines for simu1, middle: splines for simu2, right: sensor suite):

<div align=center><img src="docs/img/splines1.png" width =33%><img src="docs/img/splines2.png" width =33%><img src="docs/img/sensor12.png" width =33%></div>

For simu3 and simu4, you should change field '`config_path`' in '`ris_calib/launch/ris-calib-prog.launch`' to:

```sh
$(find ris_calib)/config/config-simu34.yaml
```

Then, we launch '`RIs-Calib`' (**before launch `RIs-Calib`, remember change the path or directory in the yaml-format configure file!**):

```sh
roslaunch ris_calib ris-calib-prog.launch
```

The calibration results (left: splines for `simu3`, middle: splines for `simu4`, right: sensor suite):

<div align=center><img src="docs/img/splines3.png" width =33%><img src="docs/img/splines4.png" width =33%><img src="docs/img/sensor3.png" width =33%></div>

You could use scripts in '`ris_calib/scripts`' to draw figures:

<details open>
    <summary><b><i>Distributions of IMU factors (accelerometer factor & gyroscope factor) in batch optimizations</i></b></summary>
    <div align=center><img src="docs/img/figures/imu_residuals.png" width =100%><img src="docs/img/figures/iter_info.png" width =100%></div>
</details>

<details open>
    <summary><b><i>Convergence performance of spatiotemporal parameters for IMUs</i></b></summary>
    <div align=center><img src="docs/img/figures/imu_simu_imu0.png" width =100%><img src="docs/img/figures/imu_simu_imu1.png" width =100%><img src="docs/img/figures/imu_simu_imu2.png" width =100%></div>
</details>

<details open>
    <summary><b><i>Convergence performance of spatiotemporal parameters for Radars</i></b></summary>
    <div align=center><img src="docs/img/figures/radar_simu_radar0.png" width =100%><img src="docs/img/figures/radar_simu_radar1.png" width =100%><img src="docs/img/figures/radar_simu_radar2.png" width =100%></div>
</details>

<details open>
    <summary><b><i>Normal equations built during one iteration</i></b></summary>
    <div align=center>
        <b><i>Control points from rotation and velocity B-splines</i></b>
        <br/>
        <img src="docs/img/lm_equ.png" width =80%>
        <br/>
        <b><i>All spatiotemporal parameters</i></b>
        <br/>
        <img src="docs/img/batch_opt_lm_equ.png" width =70%></div>
</details>

<details open>
    <summary><b><i>Rotation and velocity B-splines (left: simu3, right: simu4)</i></b></summary>
    <div align=center>
        <img src="docs/img/simu3-splines.png" width =45%>
        <img src="docs/img/simu4-splines.png" width =45%></div>
</details>
