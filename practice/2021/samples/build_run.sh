docker build -t ros-noetic-opencv .
#xhost +local:docker
docker run -it --rm \
    --env="DISPLAY" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --net=host \
    -v $(pwd):/root/catkin_ws/src/ulstu_cv \
    ros-noetic-opencv


