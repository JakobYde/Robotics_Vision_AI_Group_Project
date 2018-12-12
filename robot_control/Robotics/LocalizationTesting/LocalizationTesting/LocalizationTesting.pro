TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
            Lidar.cpp \
            Edge.cpp \
            Point.cpp \
            Grid.cpp \
            Map.cpp \
            GraphDrawer.cpp \
            Line.cpp

HEADERS +=  Lidar.h \
            Edge.h \
            Point.h \
            Grid.h \
            Map.h \
            GraphDrawer.h \
            Line.h

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv



