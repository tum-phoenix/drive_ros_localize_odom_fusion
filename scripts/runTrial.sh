#!/bin/bash

# print help and exit
function help {
 echo "Use the following arguments:"
 echo "  --trail <nr>"
 echo "  --logdir <dir>"
 echo "  --bag <bagfile>"
 echo "  --config <vehicle_config>"
 echo "  --catkin_ws <catkin_ws_dir>"
 echo ""
 exit 1
}

# parse arguments
TRAIL=-1
LOGDIR="-1"
BAGFILE="-1"
VEHICLECONFIG="-1"
CATKIN_WS="-1"

while [[ $# -gt 0 ]]
do
	key="$1"
	case $key in
		--trail)
		TRAIL="$2"
		shift # past argument
		shift # past value
		;;
		--logdir)
		LOGDIR="$2"
		shift # past argument
		shift # past value
		;;
		--bag)
		BAGFILE="$2"
		shift # past argument
		shift # past value
		;;
		--config)
		VEHICLECONFIG="$2"
		shift # past argument
		shift # past value
		;;
		--catkin_ws)
		CATKIN_WS="$2"
		shift # past argument
		shift # past value
		;;
		*)    # unknown option
		help
		;;
	esac
done

# check if all parameters are set
if [ $TRAIL = "-1" ]
then
	help
fi

if [ "$LOGDIR" = "-1" ]
then
	help
fi

if [ "$BAGFILE" = "-1" ]
then
	help
fi

if [ "$VEHICLECONFIG" = "-1" ]
then
	help
fi

if [ "$CATKIN_WS" = "-1" ]
then
	help
fi

# check if paths are valid
if [ ! -d "$LOGDIR" ]
then
  echo "$LOGDIR does not exists!"
	exit 2
fi

if [ ! -f "$BAGFILE" ]
then
  echo "$BAGFILE does not exists!"
	exit 3
fi

if [ ! -f "$VEHICLECONFIG" ]
then
  echo "$VEHICLECONFIG does not exists!"
	exit 4
fi

if [ ! -d "$CATKIN_WS" ]
then
  echo "$CATKIN_WS does not exists!"
	exit 5
fi

# source ros stuff
. /opt/ros/kinetic/setup.bash
. "$CATKIN_WS"/devel/setup.bash

# avoid port number smaller than 11320 and higher than 15320
PORT=$((11320 + $TRAIL%(15320-11320)))

# get bag duration
BAG_DURATION=$(rosbag info "$BAGFILE" | grep -oP 'duration[\s\:s\d]*\(\K(\d*)')

# try to minimize possibility of port collisions
while [ $(nc -z localhost $PORT && echo 1) ]
do
  PORT=$(($PORT+1))
done

# set indivdual port for each trial (to run multiple trails in parallel)
export ROS_MASTER_URI=http://localhost:$PORT

# start roscore
timeout $(($BAG_DURATION+6))s nice -n -5 roscore -p $PORT &> "$LOGDIR"/out.log &
sleep 2

# start odometry
timeout $(($BAG_DURATION+2))s roslaunch drive_ros_imu_odo_odometry imu_odo_odometry.launch  debug_file:=true debug_file_path:="$LOGDIR"/odom.csv vehicle_config:="$VEHICLECONFIG" &> "$LOGDIR"/out.log &
sleep 1

# play back the rosbag
nice -n -5 rosbag play "$BAGFILE" -q &> "$LOGDIR"/out.log

exit 0

