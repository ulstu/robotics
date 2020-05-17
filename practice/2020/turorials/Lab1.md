# Лабораторная работа №1

Советуем устанавливать ROS melodic на linux ubuntu 18.04. Данное сочетание ПО работает корректно и не должно вызвать каких-то трудностей в процессе выполнения лабораторных работ.

## Установка ROS

Для корректной установке ROS необходимо последовательно выполнять следующие шаги:

##### Шаг 1

Подготовительная настройка репозитория убунту. 

[Гайд по настройке]: https://help.ubuntu.com/community/Repositories/Ubuntu

##### Шаг 2

Настройка компьютера для приема ПО с ros.org:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $ (lsb_release -sc) main"> /etc/apt/sources.list.d/ros-latest.list'
```

##### Шаг 3

Настройка ключей:

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Если возникают проблемы с подключением к серверу, необходимо заменить ссылку в предыдущей команде на `hkp://pgp.mit.edu:80` или `hkp://keyserver.ubuntu.com:80`

**Или** использовать альтернативную команду:

```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

##### Шаг 4

Убедитесь, что все обновления загружены:

```
sudo apt update
```

Установите один из наборов ROS:

**Полный набор (рекомендован)**:ROS, rqt, rviz, различные библиотеки для 2D/3D симуляции и роботов:

```
sudo apt install ros-melodic-desktop-full
```

**Неполный набор**: ROS, rqt, rviz и различные библиотеки для роботов:

```
sudo apt install ros-melodic-desktop
```

Для поиска доступных пакетов можно использовать команду:

```
apt search ros-melodic
```

Для установки конкретного пакета необходимо использовать команду:

```
sudo apt install ros-melodic-PACKAGE
```

Данная команда понадобится для установки некоторых компонентов в следующих лабораторных работах.

##### Шаг 5

Добавление настроек параметров ROS в каждый сеанс bash:

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

##### Шаг 6

Установка инструментов:

```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Инициализация rosdep:

```
sudo rosdep init
rosdep update
```

Установка ROS завершена.

## Установка и настройка catkin

На данный момент catkin уже должен быть у вас установлен, если нет, то необходимо его установить:

```
sudo apt-get install ros-melodic-catkin
```

Создание catkin workspace:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

Настройка catkin окончена.

## Создание пакета для лабораторной работы

Теперь в вашей домашней папке находится папка catkin_ws. Внутри нее находится папка src. Тут будут находиться ваши пакеты.

Создание пакета:

```
cd ~/catkin_ws/src
catkin_create_pkg ИМЯ_ПАКЕТА geometry_msgs rospy
```

Далее в папке пакета создаем папку launch. В ней будут храниться launch файлы.

В папке src вашего пакета создаем python-файл со скриптом для вашей черепахи.

Собираем пакет:

```
cd ~/catkin_ws
catkin_make
```

## Разработка алгоритма движения черепахи

На текущем шаге необходимо написать скрипт, который позволит вашей черепахе перемещаться по заданной траектории. 

[Шаблон программы перемещения и launch файл]: https://github.com/ulstu/robotics_cad/tree/master/practice/2020/samples/01turtlesim

По ссылке вы сможете найти программу, в которой реализовано движение черепахи. А также там можно найти launch файл. Он позволит вам запустить все нужные узлы программы одной командой.

##### Считывание текущих координат черепахи

Узнать текущие координаты черепахи можно путем подписки на ноду с позицией черепахи:

```
pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
```

Где:

Pose - тип сообщения, который необходимо импортировать:

```
from turtlesim.msg import Pose
```

poseCallback - функция, которая будет вызвана для обработки сообщения.

Пример данной функции:

```
def poseCallback(pose_msg):
    curx = pose_msg.x
    cury = pose_msg.y
    yaw = pose_msg.theta
```

После написания скрипта для движения черепахи необходимо только запустить ее.

## Запуск приложения

##### Способ 1: без launch файла

Обратите внимание, что каждая команда запускается в новой консоли.

Запуск ros:

```
roscore
```

Запуск симуляции черепахи:

```
rosrun turtlesim turtlesim_node
```

Запуск ноды управления черепахой с клавиатуры:

```
rosrun turtlesim turtle_teleop_key 
```

Запуск скрипта движения:

```
rosrun НАЗВАНИЕ_ВАШЕГО_ПАКЕТА НАЗВАНИЕ_СКРИПТА.py
```

##### Способ 2: с launch файлом

Данной командой будут запущены все ноды, прописанные в launch файле:

```
roslaunch НАЗВАНИЕ_ВАШЕГО_ПАКЕТА НАЗВАНИЕ_LAUNCH_ФАЙЛА.launch
```

Также в таких файлах можно задавать параметры для нод.

## Полезные ссылки

[Установка ros]: http://wiki.ros.org/melodic/Installation/Ubuntu
[Туториалы по catkin]: http://wiki.ros.org/catkin/Tutorials

