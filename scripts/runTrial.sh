#!/bin/bash

# print help and exit
function help {
 echo "Use the following arguments:"
 echo "  --trial <nr>"
 echo "  --logdir <dir>"
 echo "  --bag <bagfile>"
 echo "  --config <vehicle_config>"
 echo "  --launch <launch_file>"
 echo "  --catkin_ws <catkin_ws_dir>"
 echo ""
 exit 1
}

# parse arguments
TRIAL=-1
LOGDIR="-1"
BAGFILE="-1"
VEHICLECONFIG="-1"
CATKIN_WS="-1"
LAUNCH="-1"

while [[ $# -gt 0 ]]
do
        key="$1"
        case $key in
                --trial)
                TRIAL="$2"
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
                --launch)
                LAUNCH="$2"
                shift # past argument
                shift # past value
                ;;
                --catkin_ws)
                CATKIN_WS="$2"
                shift # past argument
                shift # past value
                ;;
                *)    # unknown option
		echo "Unknown option provided: $key"
                help
                ;;
        esac
done

# check if all parameters are set
if [ $TRIAL = "-1" ]
then
	echo "Trial argument not provided."
        help
fi

if [ "$LOGDIR" = "-1" ]
then
	echo "LogDir argument not provided."
        help
fi

if [ "$BAGFILE" = "-1" ]
then
	echo "BagFile argument not provided."
        help
fi

if [ "$VEHICLECONFIG" = "-1" ]
then
	echo "VehicleConfig argument not provided."
        help
fi

if [ "$LAUNCH" = "-1" ]
then
	echo "LaunchFile argument not provided."
        help
fi

if [ "$CATKIN_WS" = "-1" ]
then
	echo "CatkinWS argument not provided."
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

if [ ! -f "$LAUNCH" ]
then
  echo "$LAUNCH does not exists!"
        exit 5
fi

if [ ! -d "$CATKIN_WS" ]
then
  echo "$CATKIN_WS does not exists!"
        exit 6
fi

# source ros stuff
. /opt/ros/kinetic/setup.bash
. "$CATKIN_WS"/devel/setup.bash

# avoid port number smaller than 11320 and higher than 15320
PORT=$((11320 + $TRIAL%(15320-11320)))

# get bag duration
BAG_DURATION=$(rosbag info "$BAGFILE" | grep -oP 'duration[\s\:s\d]*\(\K(\d*)')

# try to minimize possibility of port collisions
while [ $(nc -z localhost $PORT && echo 1) ]
do
  PORT=$(($PORT+1))
done

# set indivdual port for each trial (to run multiple trials in parallel)
export ROS_MASTER_URI=http://localhost:$PORT

# start roscore
timeout $(($BAG_DURATION+6))s nice -n -5 roscore -p $PORT &> "$LOGDIR"/out.log &
sleep 2

# start odometry
roslaunch "$LAUNCH" debug_out:=true debug_out_file_path:="$LOGDIR"/odom.csv vehicle_config:="$VEHICLECONFIG" &> "$LOGDIR"/out.log

exit 0

