#!/bin/bash

sudo ln -sf `rospack find dxl_armed_turtlebot`/scripts/ps3joy /etc/init.d/ps3joy
sudo update-rc.d ps3joy defaults 99 99 # install
#sudo update-rc.d -f ps3joy remove # uninstall
