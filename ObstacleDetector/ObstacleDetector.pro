TEMPLATE = app
QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS += -g
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

TARGET = ObstacleDetectorLoopmain
DESTDIR = /home/yyw//ObstacleDetector/bin/home/yyw//ObstacleDetector/bin
#./bin/
OBJECTS_DIR = ./

SOURCES += \
    kernel/detect_positive_obstacle.cpp \
    kernel/negativeobstacle.cpp \
    framework/ObstacleDetectorLoopmain.cpp \
    framework/alv_data.cpp

INCLUDEPATH += \
    include \
    kernel \
    framework

LIBS += \
    -L/usr/local/lib \
    -lrcs \
    -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml \

HEADERS += \
    include/alv_data.h \
    include/program.h \
    include/Lidar32DataStruct.h \
    include/detect_positive_obstacle.h \
    include/negativeobstacle.h
