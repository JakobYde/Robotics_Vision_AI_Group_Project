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
            GraphDrawer.cpp

HEADERS +=  Lidar.h \
            Edge.h \
            Point.h \
            Grid.h \
            Map.h \
            GraphDrawer.h \
            Line.h \
            RandFloat.h

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv



