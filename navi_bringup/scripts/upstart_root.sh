#!/bin/sh
ifconfig eth0 mtu 9000

. /home/ieee/fuerte/setup.sh
rosrun ps3joy ps3joy.py &

