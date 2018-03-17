# Project : SLAM-Robot Using ROS-Kobuki-Turtlebot
The project is done at University of Alberta (UoA) for the course CMPUT 412: Experimental Robotics.

## Overview
The robot builds a map of the environment and moves along a four corner loop with RGB camera.


## Dependencies

We tested our project on the following environment.
* Ubuntu 14.04
* Python 2.7.6
* ROS Indigo
* Numpy
* OpenCV-Python 2.4.8
 

How to configure Joy:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

## How to run 
### Collect map 
Run the following to change the gmapping parameters to different values. These adjustments changes how far the robot rotates and moves before a new scan is considered for inclusion in the map. It also adjusts the number of beams to skip when processing each laser scan message, and the extent of the map dimensions. The following change in parameters yielded good map results.
```
roscd turtlebot_navigation/launch
cp gmapping_demo.launch ~/copyDestinationName
roscd turtlebot_navigation/launch/includes/gmapping
cp gmapping.launch.xml
roslaunch folderName gmapping_demo.launch

<param name="lskip" value="10"/>
<param name="linearUpdate" value="0.1"/>
<param name="angularUpdate" value="0.1"/>
<param name="xmin" value="-10.0"/>
<param name="ymin" value="-10.0"/>
<param name="xmax" value="10.0"/>
<param name="ymax" value="-10.0"/>

```
Run the following to record, save and display map image. 
```
roslaunch turtlebot_bringup minimal.launch
roslaunch gmappingSavedFolderName gmapping_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
rosrun map_server map_saver -f /destinationFolderName/MapName
roscd folderMapSaved 
rosrun rviz rviz

```
At first we thought of cropping the north end of the map from our last demo as our new map for the competition. However, the quality of the north end area of the map is poor even if we tried to edit the image. Furthermore, GMapping takes sensor data (laser scan and odometry) as input, and continuously computes the map as the robot moves around. Because the sensor data is subject to noise, the map will be inaccurate. Sometimes the error is very large thus make the map unusable. One method to overcome this is to always drive a new path. Therefore, we made a new map of just the north end of the second floor. We also manually removed some noises and corrected the edges from the map using GIMP. 

<div align="center">
  <img src ="img_src/map1.pgm" width ="200"> 
</div>

<div align="center">
  <img src ="img_src/map2.pgm" width ="200"> 
</div>

### Localization and Navigation



<div align="center">
  <img src ="img_src/im5.png" width ="200"> <img src ="img_src/im5_pers.jpeg" width ="200"> <img src ="img_src/im4.png" width ="200"> <img src ="img_src/im4_pers.jpeg" width ="200">
</div>
<div align="center">
  <img src ="img_src/im3.png" width ="200"> <img src ="img_src/im3_pers.jpeg" width ="200"> <img src ="img_src/im3.png" width ="200"> <img src ="img_src/im3_pers.jpeg" width ="200">
</div>


#### Improvement


## Project Description
The steps for Lane Following are as follows:
* Convert input image into desired perspective transformation
* Convert into Grayscale(for detecting white) and HSV(for yellow)
* Form a mask with binary threshold to extract ROI for white or yellow line
* Apply morphological operator to get rid of noise
* Crop and extract a rectangular region as ROI.
* Find the moment of the region
* Declare robot location as an offset from the point of moment (we followed the right line, so offset was a distance on the left of the moment)
* Calculate error based on differnce between half the image width and robot's location.
* Run PD over the error to get angular z with a constant linear velocity x
* Repeat all the steps

For line following we can find the error with the moment directly.

<div align="center">
  <img src ="img_src/im1.png" width ="200"> <img src ="img_src/im1_pers.jpeg" width ="200"> <img src ="img_src/im1_gray.jpeg" width ="200"> <img src ="img_src/im1_hsv.jpeg" width ="200">
</div>
<div align="center">
  <img src ="img_src/im1_binary.jpeg" width ="200"> <img src ="img_src/im1_morph.jpeg" width ="200"> <img src ="img_src/im1_ROI.jpeg" width ="200"> <img src ="img_src/im1_ROIcirc.jpeg" width ="200">
</div>


## Performance Video
<div align="center">
  <a href="https://www.youtube.com/watch?v=YSfXsihIQsc"><img src="https://img.youtube.com/vi/YSfXsihIQsc/0.jpg" alt="IMAGE ALT TEXT"></a>
</div>
<div align="center">
  <a href="https://www.youtube.com/watch?v=-dGZnNCPr4A"><img src="https://img.youtube.com/vi/-dGZnNCPr4A/0.jpg" alt="IMAGE ALT TEXT"></a>
</div>
<div align="center">
  <a href="https://www.youtube.com/watch?v=XfeCy8dq6mw"><img src="https://img.youtube.com/vi/XfeCy8dq6mw/0.jpg" alt="IMAGE ALT TEXT"></a>
</div>

## Discussion
If the robot somehow turns towards the left line , it follows the track in backward direction. Sometimes it misses the line on a very sharp turn as it vanishes from the robot's view. The method also relies on the camera setting and also prone to lighting conditions where the thresholds require tuning. Our finish time on the track was 24 seconds (see video below). It also had a penalty as per the competition rule when the robot body went outside the track at a sharp corner.

## Future Work
Variable linear motion can be used instead of fixed linear motion which will help in turning. Also path planning ahead of the turn might help reduce the the linear speed and adjust turning speed for smoother motion.

## Authors

* [Nazmus Sakib](https://github.com/nsa31)
* **Vivian Ting**
## Acknowledgement 

* [Programming Robots with ROS](https://github.com/osrf/rosbook/blob/master)

