# HRVL_GBOT GazeControl
- GazeControl package for HRVL GBot

# Requirements
- hrvl_gbot_description package (INCLUDE!!!)
- hrvl_gbot_moveit_package

# HowToUse
- This package has to be used with following packages
  - hrvl_gbot_description
  - hrvl_gbot_moveit
  - dynamixel_workbench (our modified version)
- pan_tilt_control node (pan_tilt_control.py)
  - directly controls pan/tilt motors of HRVL_GBOT using dynamixel workbench services
- img_proc node (img_proc.py)
  - OpenCV-based image processing
  - Detects moving objects during gaze_transition process
  - Generate gaze directions

![image](https://user-images.githubusercontent.com/29231446/60608404-5bdb6300-9dfa-11e9-9a3d-d8fd6ff65722.png)

# Maintainer
- Prof. Jeong-Ki Yoo (HRVL @ Daejeon University)
- https://sites.google.com/view/hrvl

# License
- GNUv2

# Acknoledgement
- This work was supported by the National Research Foundation of Korea(NRF) grant funded by the Korea government(MSIT) (No. 2016R1C1B2012348).
