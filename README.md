# ROSbag to images
Convert images in ROS bagfile to readable images in PNG or JPG format

### Install

```
cd ~/catkin_ws/src
git clone https://github.com/rosbag_to_images
cd ~/catkin_ws
catkin build -c -s
source devel/setup.bash
```

### Run

Using rosrun

```
rosrun rosbag_to_images rosbag_to_images.py --topic:=<topic> --save_dir:=<save_dir> --save_name:=save_name> --bagfile:=<bagfile> --format:=<format>
```

Or roslaunch

```
roslaunch rosbag_to_images rosbag_to_images.launch --topic:=<topic> --save_dir:=<save_dir> --save_name:=save_name> --bagfile:=<bagfile> --format:=<format>
```
