resources
  https://github.com/JakobEngel/dso
  https://vision.in.tum.de/dso
  https://vision.in.tum.de/mono-dataset
  https://github.com/JakobEngel/dso_ros !!!
  https://github.com/tum-vision/mono_dataset_code

branch
  master

papers
  Direct Sparse Odometry - 2016
  A Photometrically Calibrated Benchmark For Monocular Visual Odometry  -2016
|
|
|
|
|
|
1. INSTALL (with rosbuild workspace (not catkin) / check how to translate)

  # INSTALL DEPENDENCIES
    $ sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev
    $ sudo apt-get install libopencv-dev
    $ https://github.com/stevenlovegrove/Pangolin

  # CLONE THE REPOSITORY
    $ git clone https://github.com/JakobEngel/dso.git

  # BUILD ZIPLIB
    $ sudo apt-get install zlib1g-dev
    $ cd dso/thirdparty
    $ tar -zxvf libzip-1.1.1.tar.gz
    $ cd libzip-1.1.1/
    $ ./configure
    $ make
    $ sudo make install
    $ sudo cp lib/zipconf.h /usr/local/include/zipconf.h   # (no idea why that is needed).

  # BUILD APP
    $ cd dso
  	$ mkdir build
  	$ cd build
  	$ cmake ..
  	$ make -j
|
|
|
|
|
|
2. USAGE

  # CALIBRATION SETTINGS FILE
    Pinhole fx fy cx cy 0
    in_width in_height
    "crop" / "full" / "none" / "fx fy cx cy 0"
    out_width out_height

  # LAUNCH
    $ bin/dso_dataset \
  		files=XXXXX/sequence_XX/images.zip \
  		calib=XXXXX/sequence_XX/camera.txt \
  		gamma=XXXXX/sequence_XX/pcalib.txt \
  		vignette=XXXXX/sequence_XX/vignette.png \
  		preset=0 \
  		mode=0

  # RECONFIGURE PARAMETERS
    'see on site'

  # NOTES
    'For backwards-compatibility, if the given cx and cy are larger than 1, DSO assumes all four parameters to directly be the entries of K, and ommits the above computation.'

    'the initializer is very slow, and does not work very reliably. Maybe replace by your own way to get an initialization.'

    'Use a photometric calibration'
