<a href="https://unige.it/en/">
<img src="images/genoa_logo.png" width="20%" height="20%" title="University of Genoa" alt="University of Genoa" >
</a>

>**Author: Omotoye Shamsudeen Adekoya**  
 **Email: adekoyaomotoye@gmail.com**  
 **Student ID: 5066348**  

# Outline

- Introduction
    - Robot Design
- ROS Packages
    - cluerosity_description
    - cluerosity_gazebo
    - cluerosity_slam 
    - cluerosity_navigation
    - cluerosity_moveit
    - cluerosity_unity
- System Limitations and Possible Improvements
- Installation Procedure

<div align="center">
<h1><strong>  Cluerosity </strong/></h1><br>
<img src="images/explorer.png" width="45%" height="45%" title="Cluerosity Robot equipped with LiDAR, camera and laser sensor" alt="Cluerosity Robot equipped with LiDAR, camera and laser sensor" ><img src="images/investigator.png" width="55%" height="55%" title="Cluerosity Robot equipped with LiDAR, camera and laser sensor" alt="Cluerosity Robot equipped with LiDAR, camera and laser sensor" >
</div>

# Introduction

**Cluerosity** robot is a robot designed for the purpose of ***and not limited to*** the [Experimental Robotics Lab Projects](https://github.com/Omotoye/Experimental-Robotics-Project). The Experimental Robotics Lab Projects are centered around a murder mystery board game called [Cluedo](https://en.wikipedia.org/wiki/Cluedo). The objective of the cluedo game is to *determine __who__ murdered the game's victim, __where__ the crime took place, and __what__ weapon was used*. Each player assumes the role of one of the six suspects and attempts to deduce the correct answer by strategically moving around a game board representing the rooms of a mansion and collecting clues about the circumstances of the murder from the other players. A robotics senario was created for this board game, where a robot is supposed to move into different rooms randomly or strategically acquire hints about the possible who, where and what of the murder and then generate a hypothesis that would be taken to an oracle in a set location in the mansion to confirm if the hypothesis is correct. To acquire the said hints the robot has to be fitted with sensors like cameras for perceiving the environment a manipulator to pick out hints that are above ground level and also sensors that would help with navigation. 
>The name of the game being portrayed; *Cluedo* and an inspiration from the *Curosity Rover* sent to Mars to gather clues about the existence of life, birth the name *Cluerosity :slightly_smiling_face:*

## Robot Design 

Attached below are fun _.gifs_ files that shows the timeline of the design of the robot; from the use of primitive shapes to meshes.

<br/>

<div align="center">
<img src="images/version-1.gif" width="50%" height="50%" title="Cluerosity Robot equipped with LiDAR, camera and laser sensor" alt="Cluerosity Robot equipped with LiDAR, camera and laser sensor" ><img src="images/version-2.gif" width="50%" height="50%" title="Cluerosity Robot equipped with LiDAR, camera and laser sensor" alt="Cluerosity Robot equipped with LiDAR, camera and laser sensor" >
<img src="images/version-3.gif" width="50%" height="50%" title="Cluerosity Robot equipped with LiDAR, camera and laser sensor" alt="Cluerosity Robot equipped with LiDAR, camera and laser sensor" ><img src="images/version-4.gif" width="50%" height="50%" title="Cluerosity Robot equipped with LiDAR, camera and laser sensor" alt="Cluerosity Robot equipped with LiDAR, camera and laser sensor" >
</div>

### Sensor Housing

<img align="right"  src="images/sensor_housing.png" width="15%">

The is a box standing on a gimbal, it houses a RGB, RGB-D camera and a laser sensor. The sensor housing gimbal can *pan 360 degrees* and *tilt 20 degree* up/down. The laser sensor is used by the slam packages to map the environment; which is then used by the navigation package to navigate to a goal point. The depth camera can be used for getting point cloud data of the environment to generate 3D map of the environment. The rgb camera is used for getting image data which would be used to percieve hint data in the robotics cluedo senario. 

### Robot Manipulator 

<img align="right"  src="images/mani_camera.png" width="15%">

This manipulator carries a camera for hint perception and a shelock link pointer to also acquire hint in the second version of the experimental robotics project. The end effector carrying the camera and sherlock link has a continuous joint that allows it turn 360 degrees. The manipulator link tree has 4 revolute joints leading to the end effect, so this helps with redundancy and allows the sherlock link reach more point in the space in front of the robot. 

<br><br>
### Velodyne LiDAR

<img align="right"  src="images/velodyne.png" width="15%" >

The Velodyne LiDAR is not actually used for any part of the project, so for now, it's just there as a prop and serve no actual use. Of course if the project is to be adapted for a different project that might require such sensor, a simple edit of the robot description would do the trick. 

<br><br>

# ROS Packages
Several packages were implemented for different functionalities required by the robot. Below is a general overview of each of the packages. 

## cluerosity_description

This is the package that contains the **urdf** of the package and all the files required for the urdf description of the robot. _Xacro_ files were implemented to modularize the urdf of this robot and then the urdf was generated from the xacro files. This package feature two launch files `display.launch` and `description.launch`. **_After following the installation procedure_** of this package run the launch file below to view the robot. 

```bash
roslaunch cluerosity_description display.launch # the default robot that would be launched is the explorer robot
```

_to launch the robot with the manipulator; **investigator**, use the launch file below_

```bash
roslaunch cluerosity_description display.launch type:=investigator 
```

The launch files above would load the robot description into the parameter server and the launch **rviz** with a customized configuration showing the robot in it; if you just want to load the robot description into the parameter server, use the launch file below. 

```bash
roslaunch cluerosity_description description.launch
```

same goes here for the type of robot, if you require the robot with the manipulator, use the launch file below. 

```bash
roslaunch cluerosity_description description.launch type:=investigator
```