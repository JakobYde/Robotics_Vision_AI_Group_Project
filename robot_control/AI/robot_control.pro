TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    FuzzyBugController.cpp \
    LaserScanner.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

unix:!macx: LIBS += -L/home/mnj/rb-rca5-group2/fuzzylite-6.0-linux64/fuzzylite-6.0/release/bin/ -lfuzzylite-static
# unix:!macx: LIBS += -L$$PWD/../../fuzzylite-6.0-linux64/fuzzylite-6.0/release/bin/ -lfuzzylite-static

INCLUDEPATH += $$PWD/../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite
DEPENDPATH += $$PWD/../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite

DISTFILES += \
    fuzzybugcontroller.fll

HEADERS += \
    FuzzyBugController.h \
    LaserScanner.h
