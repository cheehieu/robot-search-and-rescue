robot-search-and-rescue
=======================

<img src="http://niftyhedgehog.com/robot-search-and-rescue/images/robot_closed.jpg">

## Overview
Used Lego bricks to build a robot to efficiently navigate a course and perform search-and-rescue tasks. Sensors/actuators included sonar module, IR sensor, resistive contact sensor, compass, web-camera, servo-motors, and gripper. Developed algorithms for object recognition, vision processing, Monte-Carlo localization particle filtering, minimizing sensor error.

This project was developed in the fall of 2010 for USC's Introduction to Robotics course (CSCI-445) taught by professor Laurent Itti. My team members included Suk-jin "Justin" Lee and Asif Ahmed, under the supervision of TA, Chin-Kai Chang.

## Hardware
* LEGOs! to build the body, gripper (actuator)
* DC motors, servo motors, gear ratios
* Sonar module (long range)
* IR proximity sensor (short range)
* Resistive contact sensors for touch detection
* Compass for directional navigation
* Webcam with white balance disabled for vision

## Software
My major contribution for this project was implementing the state machine and controlling the overall behavior of our robot. I pretty much had a hand in every part of the code for interfacing and testing/debugging all of our sensors. These include an IR sensor for wall following, sonar for moving forward and updating the particle filter, compass for semi-accurate turning and maintaining a forward heading, and the camera for “object recognition.” I also wrote the state machine that allowed the robot to find targets and plan paths to a particular destination.  

More specifically I wrote:

* Finding median sensor readings to improve accuracy
* Functions for turning based on compass readings (orient North, South, East, or West)
* PID control for moving forward/backward (using the compass to maintain a heading)
* Functions to grab and release the green target with servo-controlled “pincer”
* PID control for wall-following using the IR sensor and sonar
* Particle/Kalman filter functions for moving and updating the particles and interfacing them with the mapImg
* Functions to find pink and orange objects (color segmenter, connected components, PID)
* Functions to plan a path to nearest purple dock, and search for green target

## Demo
Embedded YouTube video

## Team
Left to right: Asif, Hieu, Justin, robot, Kai
<img src="http://niftyhedgehog.com/robot-search-and-rescue/images/team5.jpg">

## Lessons Learned
I’m an electrical engineering major, so my past experience in robotics has focused more on the hardware as opposed to the software.  However, this final project gave me the opportunity to work more with programming and software control.It was nice to get a computer science perspective in robotics because now I understand why robotics is so difficult; when writing code to control a robot, things never go according to plan. You end up spending a great deal of time testing and debugging how things will transfer from a deterministic environment to the physical world. The motors were heavily dependent on the battery voltage and were not synchronized with each other. The sensors were very inaccurate; the compass readings were not divided equally by degree and the sonar did not work well over long distances. These were all challenges my group had to overcome, and we have spent countless hours trying to meet the three milestones, let alone finish the final project.  

Next time, I would try to improve the speed of our robot. The path-planning algorithm now divides the map into a 13x13 grid (to account for walls) and finds its way to a desired coordinate based off this. However, the particle filter divides the map into centimeters (a 361x361 grid) so there is a continuous translation between the path-planning and particle filtering.  Thus, our robot moves in small increments—once for each coordinate—and this takes a relatively long time for our robot to move to its destination because after moving, the robot must stop to take measurements for localization and navigation.  While this works to accomplish the task, it is not optimal. Next time, I could try to optimize the software and remove sensor delays to improve the latency of our robot, and make it seem like the robot is moving continuously. Or perhaps somehow integrate the path-planning into the particle filter.
