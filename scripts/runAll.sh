#!/bin/bash

# parameter
DBHOST="localhost"
DBPORT="1234"
DBDIR="/home/fabian/mongodb/"
EVALS_MAX="40"
WORKER_MAX=40


# get current date and time as experiment name
EXPERIMENT=$(date +%Y-%m-%d-%H-%M-%S)
echo "Start experiment $EXPERIMENT ..."

# get the script folder
SCRIPT_FOLDER="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


echo "  Start MongoDB ..."
mongod --dbpath $DBDIR --port $DBPORT &> /dev/null &
sleep 1

echo "  Start kalmanTuning.py ..."
python3 $SCRIPT_FOLDER/kalmanTuning.py --mongohost $DBHOST --mongoport $DBPORT --experiment $EXPERIMENT --max_evals $EVALS_MAX &> /tmp/kalmanTuningMain.log &
sleep 1


WORKER=0
while [ $WORKER -lt $WORKER_MAX ]
do
  echo "  Start worker $WORKER ..."
  hyperopt-mongo-worker --mongo=$DBHOST:$DBPORT/$EXPERIMENT --poll-interval=1 --reserve-timeout=5 --max-consecutive-failures=4 &> /tmp/kalmanTuningWorker$WORKER.log &
  WORKER=$(($WORKER+1))
  sleep 2
done


