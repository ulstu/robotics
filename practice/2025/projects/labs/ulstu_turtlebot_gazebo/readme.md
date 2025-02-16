## Main commands

```bash
ros2 run tf2_tools view_frames
```

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=my_map.yaml
```

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True 
```

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=my_map.yaml params_file:=./src/ulstu_turtlebot_gazebo/resource/params.yaml
```