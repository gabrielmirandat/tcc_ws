// roda stream de vídeo publicando em camera/image_raw
$ roslaunch video_stream_opencv video_file.launch

// roda ORB_SLAM ouvindo em camera/image_raw
$ roslaunch ORB_SLAM ExampleGroovyOrNewer.launch

// roda calibrador de câmera ouvindo em camera/image_raw
$ rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.108 image:=/camera/image_raw camera:=/camera

// corrige bags fechados antes da hora
$ rosbag reindex --output-dir=reindexed *.bag.active

// roda bags em sequencia com fator x3 começando pausado
$ rosbag play -r 3 --pause recorded1.bag recorded2.bag

// roda vizualizador das imagens sendo publicadas em camera/image_raw
$ rosrun image_view image_view image:=/camera/image_raw
