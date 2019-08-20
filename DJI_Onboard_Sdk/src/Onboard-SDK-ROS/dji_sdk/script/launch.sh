#!/bin/sh
### BEGIN INIT INFO
# Provides:          land.sh
# Required-start:    $local_fs $remote_fs $network $syslog
# Required-Stop:     $local_fs $remote_fs $network $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: starts the svnd.sh daemon
# Description:       starts svnd.sh using start-stop-daemon
### END INIT INFO

#gnome-terminal -t "m600_air_node" -x bash -c "roscore;exec bash";
gnome-terminal -t "onboard_sdk_node" -x bash -c "source ~/DJI_Onboard_Sdk/devel/setup.bash;roslaunch dji_sdk sdk.launch;exec bash";
gnome-terminal -t "m600_air_node" -x bash -c "source ~/DJI_Onboard_Sdk/devel/setup.bash;rosrun dji_sdk m600_air;exec bash";
gnome-terminal -t "m600_gim_driver" -x bash -c "source ~/DJI_Onboard_Sdk/devel/setup.bash;rosrun dji_sdk m600_gimbal;exec bash";
