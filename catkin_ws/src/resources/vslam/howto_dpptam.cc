resources
  https://github.com/alejocb/dpptam

branch
  master

papers
  DPPTAM: Dense Piecewise Planar Tracking and Mapping from a Monocular Sequence 2015
|
|
|
|
|
|
1. INSTALL (with rosbuild workspace (not catkin) / check how to translate)

  # INSTALL DEPENDENCIES
    $ sudo apt-get install ros-indigo-pcl-ros
    $ sudo apt-get install libboost-all-dev

  # CLONE THE REPOSITORY
    $ git clone  https://github.com/alejocb/dpptam.git

  # BUILD SUPERPIXELS
    $ cd root/catkin_workspace/src/dpptam/ThirdParty/segment
    $ make

  # BUILD APP
    $ catkin_make --pkg dpptam
|
|
|
|
|
|
2. USAGE

  # CALIBRATION SETTINGS FILE (see 'dpptam/src/data.yml')
    'cameraMatrix'
    'distCoeffs'
    'camera_path:"/image_raw"'

  # LAUNCH
    $ cd root/catkin_workspace
    $ rosrun dpptam dpptam

  # TO RUN BAG
    $ rosbag play lab_unizar.bag
    $ rosbag play lab_upenn.bag

  # RECONFIGURE PARAMETERS
    'see on site'

  # NOTES
    'The initialization is performed assuming that the first map is a plane parallel to the camera'

    'In order to exploit the benefits of the use of superpixels for low texture areas we provide the following hint: Try to 'see' the low texture area in at least 3 different views with enough parallax, so the supeprixel can be matched in multiple views. Notice that superpixels triangulation need a larger parallax than feature/photometric triangulation.'
