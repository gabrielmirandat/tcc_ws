# compile
$ source ~/ros/tcc_ws/rosbuild_ws/setup.bash
!$ cd ~/ros/tcc_ws/rosbuild_ws/src
!$ rosmake ORB_SLAM
$ cd ~/ros/tcc_ws/rosbuild_ws/src/ORB_SLAM/build
$ make
$ roscore

# run bag paused
$ rosbag play --pause ~/ros/tcc_ws/#resources/bags/sensor_data_2017-11-09-14-29-09.bag

# run visualizer images and map
$ rosrun image_view image_view image:=/ORB_SLAM/Frame _autosize:=true
$ rosrun rviz rviz -d Data/rviz.rviz

# run vslam
$ rosrun ORB_SLAM ORB_SLAM  Data/ORBvoc.txt ../../../#resources/calibration/orbslam_pioneer.yaml
