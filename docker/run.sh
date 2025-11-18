xhost +
PROJECT_DIR="/path/to/your/ros2_ws"
DATASET_DIR="/path/to/your/dataset"

docker run --rm -it --ipc=host --net=host --privileged \
    --env="DISPLAY" \
    --user $(id -u):$(id -g) \
    --volume="$PROJECT_DIR:/root/ros2_ws/" \
    --volume="$DATASET_DIR:/root/data" \
    garlileo:latest
xhost -
