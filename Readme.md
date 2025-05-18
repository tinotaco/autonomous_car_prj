# Autonomous Car Project

In this project, a small RC car should be able to drive through an apartment building autonomously.



## Current Progress:

At the moment the RC car can:

    - Be controlled by a Playstation controller through the use of the Human Interface Device
    - Send control inputs to an Arduino which controls a servo for steering angle and a motor for acceleration and deceleration.
    - Implement an SDK to interpret sensor values sent from an RP LiDAR A1M8 and publish the scan using ROS2 messages. The Laserscan can be displayed in RViz.
    - Implement Hector SLAM (currently in progress)