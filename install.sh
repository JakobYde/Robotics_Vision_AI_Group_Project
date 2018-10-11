#!/bin/bash
#Tjek sudo er kørt
sudo apt-get update
sudo apt-get upgrade
echo Git install
sudo apt install -y git
echo Gazebo dev  install
sudo apt install -y libgazebo9-dev
echo OPencv install
sudo apt install -y libopencv-dev
echo QT5 qmake install
sudo apt install -y qt5-qmake
echo QTcreator install
sudo apt install -y qtcreator
echo Gazebo install
sudo apt install -y gazebo9
echo OPenctm install
sudo apt intall -y openctm-tools
echo Pstoedit install
sudo apt install -y pstoedit
echo Postrace install
sudo apt install -y potrace
echo OPenSCAD install
sudo add-apt-repository ppa:openscad/releases
sudo apt-get install -y openscad
