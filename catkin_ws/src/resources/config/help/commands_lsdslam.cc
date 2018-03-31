# compile
$ source ~/ros/tcc_ws/rosbuild_ws/setup.bash
$ cd ~/ros/tcc_ws/rosbuild_ws/src
$ rosmake lsd_slam
$ roscore

# run bag paused
$ rosbag play --pause ~/ros/tcc_ws/#resources/bags/sensor_data_2017-11-09-14-29-09.bag

# run visualizer
$ rosrun lsd_slam_viewer viewer

# run vslam
$ rosrun lsd_slam_core live_slam /image:=/camera/image_raw _calib:=/home/gabriel/ros/tcc_ws/#resources/calibration/lsdslam_pioneer.cfg
