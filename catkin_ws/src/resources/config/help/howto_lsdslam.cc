resources
  https://github.com/tum-vision/lsd_slam
  https://vision.in.tum.de/research/vslam/lsdslam?redirect=1
  http://vision.in.tum.de/lsdslam

branch
  catkin

papers
  LSD-SLAM: Large-Scale Direct Monocular SLAM - ECCV 14
  Semi-Dense Visual Odometry for a Monocular Camera - ICCV 13
|
|
|
|
|
|
1. INSTALL (with rosbuild workspace (not catkin) / check how to translate)

  # INSTALL DEPENDENCIES
    $ sudo apt-get install ros-indigo-libg2o ros-indigo-cv-bridge liblapack-dev libblas-dev freeglut3-dev libqglviewer-dev libsuitesparse-dev

  # CLONE THE REPOSITORY
    $ git clone https://github.com/tum-vision/lsd_slam.git lsd_slam

  # BUILD APP
    $ rosmake lsd_slam


  # CREATE ROSBUILD WORKSPACE
    $ sudo apt-get install python-rosinstall
    $ mkdir ~/rosbuild_ws
    $ cd ~/rosbuild_ws
    $ rosws init . /opt/ros/indigo
    $ mkdir package_dir
    $ rosws set ~/rosbuild_ws/package_dir -t .
    $ echo "source ~/rosbuild_ws/setup.bash" >> ~/.bashrc
    $ bash
    $ cd package_dir

1.1 (OPTIONAL) ENABLE LARGE LOOP DETECTION WITH OPENFABMAP

  # UNCOMMENT LINES IN 'lsd_slam_core/CMakeLists.txt'
    #add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
    #include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
    #add_definitions("-DHAVE_FABMAP")
    #set(FABMAP_LIB openFABMAP )

  # HABILITATE NONFREE MODULE IN OPENCV 2.4.8 ~ 2.4.12
|
|
|
|
|
|
2. USAGE
  'lsd_slam_core' full slam system
  'lsd_slam_viewer' 3D visualization

2.1 LSD_SLAM_CORE

  # OPENCV CALIBRATION CAMERA MODEL (samples in 'lsd_slam_core/calib')
    fx fy cx cy k1 k2 p1 p2
    inputWidth inputHeight
    "crop" / "full" / "none" / "e1 e2 e3 e4 0"
    outputWidth outputHeight

  # START lsd_slam_core
    $ rosrun lsd_slam_core live_slam /image:=<yourstreamtopic> _calib:=<calibration_file>

  # KEYS
    'r' reset
    'd/e' debug displays
    'o' screen info
    'm' save map as images
    'p' force new constraints
    'l' force relocalizer (track is lost)

  # RECONFIGURE PARAMETERS
    $ rosrun rqt_reconfigure rqt_reconfigure
    'see the list on site'

  # NOTES
    'use global shutter camera'
    'camera with wide field of view'
    'at least 30fps fr'
    '640x480 images is ideal'
    'adjust minUseGrad and cameraPixelNoise'
    ...

2.2 LSD_SLAM_VIEWER

  # START lsd_slam_viewer (can export cloud as .ply)
    $ rosrun lsd_slam_viewer viewer

  # RECORD, PLAY, BEGIN WITH CLOUD
    $ rosbag record /lsd_slam/graph /lsd_slam/keyframes /lsd_slam/liveframes -o file_pc.bag
    $ rosbag play file_pc.bag
    $ rosrun lsd_slam_viewer viewer file_pc.bag

  # KEYS
    'r' reset all cloud
    'w' info
    'p' export point cloud

  # RECONFIGURE PARAMETERS
    'see the list on site'
