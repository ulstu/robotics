# Лабораторная работа №2

## Установка пакетов gazebo

```
sudo apt-get install ros-melodic-gazebo-pkgs
```

Если по какой-то причине у вас не установлено gazebo, то установить его можно комадной:

```
curl -sSL http://get.gazebosim.org | sh
```

## Решение часто возникаевых ошибок

### rosdep: command not found

Решение:

```
sudo apt-get install python-pip
sudo pip install -U rosdep
sudo rosdep init
rosdep update
```

### Failed to create the dwa_local_planner/DWAPlannerROS planner

Решение:

```
sudo apt-get install ros-melodic-dwa-local-planner
```

## Создание своего мира в gazebo

Запуск gazebo:

```
gazebo
```

После запуска у вас откроется пустой мир. Стены и прочие элементы можно создавать с помощью вкладки insert.

Если у вас не отображаются стандартные модели, то необходимо добавить путь до них "/home/ИМЯ/.gazebo/models". 

Далее необходимо просто выбирать объект из стандартных моделей и устанавливать его. В меню инструментов на верхней панели можно найти инструменты для перемещения, вращения и изменения размеров.

После создания мира его необходимо сохранить.

File -> Save World As.

## Установка turtlebot3

```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws && catkin_make
```

## Построение карты

[Примеры launch файлов]: https://github.com/ulstu/robotics_cad/tree/master/practice/2020/samples/02navigation/launch

На этом этапе необходимо написать launch файл, который будет запускать ваш мир, робота нужной модели, ноду управления роботом с клавиш и slam алгоритм.

Далее необходимо с помощью клавиш пройти роботом всю карту. 

Карта сохраняется командой:

```
rosrun map_server map_saver -f ~/map
```

## Перемещение с помощью карты

[Примеры ко 2 лабораторной работе]: https://github.com/ulstu/robotics_cad/tree/master/practice/2020/samples/02navigation

В примерах находятся наброски скрипта, позволяющего роботу перемещаться к указанным координатам.

Его необходимо доработать.

Далее необоходимо написать launch файл, который запускает симуляцию мира и робота, ваш скрипт и навигацию робота.

Запуск launch-файла описан в методических указаниях к лабораторной работе №1/ 

## Полезные ссылки

[Документация turtlebot3]: http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
[Документация gazebo]: http://gazebosim.org/

