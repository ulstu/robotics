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

Примеры на python + ROS:
1. Turtlesim
2. Turtlebot3 simple move
3. Turtlebot3 SLAM (launch files) + planning (amcl launch + python)
4. PID control + lidar data in python
5. OpenCV + python in ROS


Лекции:
1. Введение в python
2. Введение в ROS
3. Введение в ROS
4. ПИД-регулятор. Основы построения карт и методы планирования пути. PID в turtlesim
5. Автоматные модели роботов.
6. SLAM + AMCL
5. Фильтр частиц. 


