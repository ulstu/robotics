# Ссылки на статьи по установке окружения

Рекомендуемая для установки версия ROS - melodic

## 1. Установка ROS
http://wiki.ros.org/kinetic/Installation/Ubuntu

## 2. Установка подсистемы компиляции catkin для ROS
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
```

## 3. Установка пакетов для симуляции turtlebot3 в gazebo
```
sudo apt install python-roslaunch
sudo apt-get install ros-melodic-kobuki-*
sudo apt-get install ros-melodic-turtlebot3
sudo apt-get install ros-melodic-turtlebot3-simulations
sudo apt-get install ros-melodic-rviz

export TURTLEBOT3_MODEL="waffle"
roslaunch turtlebot3_fake turtlebot3_fake.launch

```
Установка pip для управления пакетами python: https://pip.pypa.io/en/stable/installing/

