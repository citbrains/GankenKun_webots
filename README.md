# GankenKun_webots

This repository contains a proto file and a walking program to simulate GankenKun in webot.

It has the following features.

- Parallel links are used.  
I think it is rare to see a model of humanoid robot that simulates parallel links.

- It is a simple model to reduce the load on the computer.  
It can be simulated almost in real time on a notebook PC without GPU.  
(webots is a great simulator!)

- It can be controlled by the same program as the real robot.  
The robot can be controlled with the same program as the real robot, including parameters, without changing the program of the real robot.  
(We were surprised!)  

The environment including webot can be found below.  
https://github.com/citbrains/webots

dockhub  
https://hub.docker.com/r/citb/webots_citbrains

We participated in RoboCup2021 and got 4th place.  
https://humanoid.robocup.org/rc2021/

[How to build the program for walking](https://github.com/citbrains/GankenKun_webots/wiki/webots%E3%81%A7%E8%A9%A6%E5%90%88%E3%81%AE%E7%92%B0%E5%A2%83%E3%82%92%E4%BD%9C%E6%88%90%E3%81%99%E3%82%8B%E6%89%8B%E9%A0%86)

![image](https://user-images.githubusercontent.com/5755200/115998122-cc332400-a620-11eb-90d5-0e83166787e8.png)

# GankenKun_walking

A sample program of walking control using preview control  
Ported from [GankenKun_pybullet](https://github.com/citbrains/GankenKun_pybullet)  

Install
- webots R2021b https://www.cyberbotics.com/
```
sudo apt install python3-pip
pip3 install numpy transforms3d scipy control
git clone https://github.com/citbrains/GankenKun_webots
```

Execute
```
cd ~/GankenKun_webots/worlds
webots --batch walking.wbt
```


[![video](http://img.youtube.com/vi/lQauhJC1u4o/0.jpg)](https://www.youtube.com/watch?v=lQauhJC1u4o)

# collect_keypoints

Automatic annotation system for collecting key point images for posture estimation

Install
- webots R2021b https://github.com/cyberbotics/webots/releases/tag/R2021b
```
sudo apt install python3-pip
pip3 install -U pip
pip3 install numpy transforms3d scipy opencv-python
git clone https://github.com/citbrains/GankenKun_webots
```

Execute
```
cd ~/GankenKun_webots/worlds
webots --batch collect_keypoints.wbt
```

[![IMAGE](http://img.youtube.com/vi/kidIe4T4D5M/0.jpg)](https://youtu.be/kidIe4T4D5M)
