# compile
$ source ~/ros/tcc_ws/catkin_ws/devel/setup.bash
$ cd ~/ros/tcc_ws/catkin_ws
$ catkin_make --pkg dpptam
$ roscore

# run bag paused
$ rosbag play --pause ~/ros/tcc_ws/#resources/bags/sensor_data_2017-11-09-14-29-09.bag

# run visualizer images and map
$ rosrun image_view image_view image:=/dpptam/camera/image
$ rosrun rviz rviz

# run vslam
$ rosrun dpptam dpptam
