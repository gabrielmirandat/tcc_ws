resources
  http://webdiis.unizar.es/~raulmur/orbslam/

branch
  master

papers
  ORB-SLAM: A Versatile and Accurate Monocular SLAM System - 2015 IEEE
  Bags of Binary Words for Fast Place Recognition in Image Sequences - 2012
|
|
|
|
|
|
1. INSTALL

  # INSTALL DEPENDENCIES
    $ sudo apt-get install libboost-all-dev
    $ sudo apt-get install libeigen3-dev

  # CLONE THE REPOSITORY
    $ git clone https://github.com/raulmur/ORB_SLAM.git ORB_SLAM

  # GENERAL CONFIG
    remove the depency of opencv2 in the manifest.xml

  # ADD LINE IN '.bashrc'
    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH_TO_PARENT_OF_ORB_SLAM

  # BUILD G2O IN 'Thirdparty/g2o/'
    $ mkdir build
    $ cd build
    $ cmake .. -DCMAKE_BUILD_TYPE=Release
    $ make

  # BUILD DBOW2 IN 'Thirdparty/DBoW2/'
    $ mkdir build
    $ cd build
    $ cmake .. -DCMAKE_BUILD_TYPE=Release
    $ make

  # BUILD APP
    $ mkdir build
    $ cd build
    $ cmake .. -DROS_BUILD_TYPE=Release
    $ make
|
|
|
|
|
|
2. USAGE

  # CALIBRATION SETTINGS FILE (see 'Data/Settings.yaml')

  # LAUNCH ORBSLAM
    $ roscore
    $ rosrun ORB_SLAM <ORB_SLAM PATH_TO_VOCABULARY> <PATH_TO_SETTINGS_FILE>

  # LAUNCH FRAMES VISUALIZER
    $ rosrun image_view image_view image:=/ORB_SLAM/Frame _autosize:=true

  # LAUNCH MAP VISUALIZER
    $ rosrun rviz rviz -d Data/rviz.rviz

  # RUN ALL WITH ROSLAUNCH
    $ roslaunch ExampleGroovyOrNewer.launch

  # TO RUN BAG
    $ rosbag play --pause Example.bag

  # FAILURES
    'no translation at system initialization (or too much rotation)'
    'pure rotations in exploration'
    'low texture environments'
    'many (or big) moving objects, especially if they move slowly'
    ...

  # NOTES
    'The system is able to initialize from planar and non-planar scenes. In the case of planar scenes, depending on the camera movement relative to the plane, it is possible that the system refuses to initialize, see the paper [1] for details'
