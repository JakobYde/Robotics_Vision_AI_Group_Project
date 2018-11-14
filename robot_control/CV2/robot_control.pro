TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

unix:!macx: LIBS += -L$$PWD/../../fuzzylite-6.0-linux64/fuzzylite-6.0/release/bin/ -lfuzzylite-static

INCLUDEPATH += $$PWD/../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite
DEPENDPATH += $$PWD/../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite
