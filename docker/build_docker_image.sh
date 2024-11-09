#!/bin/bash
#docker rmi irlab-image
docker build --build-arg NUM_THREADS=8 --rm -t unreal-engine-ros2-image .
