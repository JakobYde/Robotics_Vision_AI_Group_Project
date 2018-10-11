#!/bin/bash
#Tjek sudo er kørt
sudo apt-get update
sudo apt-get upgrade
echo Git install
sudo apt install git
echo Gazebo dev  install
sudo apt install libgazebo9-dev
echo OPencv install
sudo apt install libopencv-dev
echo QT5 qmake install
sudo apt install qt5-qmake
echo QTcreator install
sudo apt install qtcreator
echo Gazebo install
sudo apt install gazebo9
echo OPenctm install
sudo apt intall openctm-tools
echo Pstoedit install
sudo apt install pstoedit
echo Postrace install
sudo apt install potrace
echo OPenSCAD install
sudo add-apt-repository ppa:openscad/releases
sudo apt-get install openscad
