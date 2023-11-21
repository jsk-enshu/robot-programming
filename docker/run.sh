#!/bin/bash

docker run -it -p 8080:8080 \
       --name ros-novnc-web \
       jsk-enshu/ros-novnc-web:latest
