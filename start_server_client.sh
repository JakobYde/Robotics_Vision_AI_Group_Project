#!/bin/bash
bash gazebo_client.sh &
jobs
bash gazebo_server.sh $1
