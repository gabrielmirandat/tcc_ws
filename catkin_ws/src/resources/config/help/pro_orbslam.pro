#To make a project qtable, just add a .pro file with the following tags
#run with  qmake -o Makefile border.pro to generate the makefile
#run make

INCLUDEPATH= /opt/ros/indigo/include
DEPENDPATH = include

#TARGET  = main

#QMAKE_CXX = ccache g++

CONFIG -= app_bundle

HEADERS += include/*.h

SOURCES += src/*.cc

LIBS    += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_features2d -lopencv_nonfree -lboost_timer -lboost_system

#QMAKE_LFLAGS += -c
