0. Build

$ cd build
$ make

1. Launch ORB-SLAM from the terminal

terminal1: $ roscore

terminal2: $ rosrun ORB_SLAM ORB_SLAM PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE

'PATH_TO_VOCABULARY' - path to ORB vocabulary
				     - absolute or relative to ORB_SLAM root
				     - example vocabulary provided in 'ORB_SLAM/Data/ORBvoc.txt.tar.gz' needed to uncompress

'PATH_TO_SETTINGS_FILE' - camera calibration and setting parameters from a YAML file
					    - absolute or relative to ORB_SLAM root
					    - you should provide the settings for your camera
					    - use the camera calibration model of OpenCV
					    - example in 'Data/Settings.yaml'

2. Processed frames published to topic '/ORB_SLAM/Frame'. To see them use:

terminal3: $ rosrun image_view image_view image:=/ORB_SLAM/Frame _autosize:=true

3. Map published to topic '/ORB_SLAM/Map'
   Current camera pose sent by /tf in frames '/ORB_SLAM/Camera'
   Global world coordinate origin sent by /tf in frames '/ORB_SLAM/World'

   To see the map:

terminal4: $ rosrun rviz rviz -d Data/rviz.rviz

4. Images received from topic '/camera/image_raw'
   You can play your rosbag or start your camera node
   To generate a bag from individual images use 'https://github.com/raulmur/BagFromImages'

5. To run all directly, ORB_SLAM, image_view and rviz, use

terminal0: $ roslaunch ExampleGroovyOrNewer.launch

6. Rosbag example 'http://webdiis.unizar.es/~raulmur/orbslam/downloads/Example.bag.tar.gz'
   To play the rosbag:

terminal5: $  rosbag play --pause Example.bag