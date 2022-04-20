<a href="https://unige.it/en/">
<img src="images/genoa_logo.png" width="20%" height="20%" title="University of Genoa" alt="University of Genoa" >
</a>

>**Author: Omotoye Shamsudeen Adekoya**  
 **Email: adekoyaomotoye@gmail.com**  
 **Student ID: 5066348**  

# Outline

- Introduction
- Tech Specs
- ROS Packages
- System Limitations and Possible Improvements
- Installation and running procedure

<div align="center">
<h1><strong>  Cluerosity </strong/></h1><br>
<img src="images/explorer.png" width="45%" height="45%" title="Cluerosity Robot equipped with LiDAR, camera and laser sensor" alt="Cluerosity Robot equipped with LiDAR, camera and laser sensor" ><img src="images/investigator.png" width="55%" height="55%" title="Cluerosity Robot equipped with LiDAR, camera and laser sensor" alt="Cluerosity Robot equipped with LiDAR, camera and laser sensor" >
</div>

# Introduction

**Cluerosity** robot is a robot designed for the purpose of ***and not limited to*** the [Experimental Robotics Lab Projects](https://github.com/Omotoye/Experimental-Robotics-Project). The Experimental Robotics Lab Projects are centered around a murder mystery board game called [Cluedo](https://en.wikipedia.org/wiki/Cluedo). The objective of the cluedo game is to *determine __who__ murdered the game's victim, __where__ the crime took place, and __what__ weapon was used*. Each player assumes the role of one of the six suspects and attempts to deduce the correct answer by strategically moving around a game board representing the rooms of a mansion and collecting clues about the circumstances of the murder from the other players. A robotics senario was created for this board game, where a robot is supposed to move into different rooms randomly or strategically acquire hints about the possible who, where and what of the murder and then generate a hypothesis that would be taken to an oracle in a set location in the mansion to confirm if the hypothesis is correct. To acquire the said hints the robot has to be fitted with sensors like cameras for perceiving the environment a manipulator to pick out hints that are above ground level and also sensors that would help with navigation. 
>The name of the game being portrayed; *Cluedo* and an inspiration from the *Curosity Rover* sent to Mars to gather clues about the existence of life, birth the name *Cluerosity :slightly_smiling_face:*
