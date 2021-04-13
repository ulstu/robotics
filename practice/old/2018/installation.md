# Ссылки на статьи по установке окружения

Рекомендуемая для установки версия ROS - kinetic

## 1. Установка ROS
http://wiki.ros.org/kinetic/Installation/Ubuntu

## 2. Установка Gazebo
http://gazebosim.org/tutorials?tut=install_ubuntu

```
sudo apt install python-roslaunch
```
Установка pip для управления пакетами python: https://pip.pypa.io/en/stable/installing/

## 4. Установка подсистемы компиляции catkin для ROS
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
```


