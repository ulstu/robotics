# Основные команды
## Сохранение карты
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ulstu_turtlebot/resource/my_map
```

## Запуск решения - навигация по построенной карте
```bash
ros2 launch ulstu_turtlebot robot_launch.py map:=~/ros2_ws/src/ulstu_turtlebot/resource/my_map.yaml
```