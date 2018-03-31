#!/bin/bash  

echo "calibrating camera - run video first"  
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.108 image:=/camera/image_raw camera:=/camera
