#!/bin/bash -e
echo "From Context: $PWD"

BASEDIR=$(dirname "$0")
echo "Using Dockerfile: $BASEDIR"

docker build -f $PWD/$BASEDIR/Dockerfile -t metagoto/ros2-basic-examples_hello_world:humble-v1.0 $BASEDIR
