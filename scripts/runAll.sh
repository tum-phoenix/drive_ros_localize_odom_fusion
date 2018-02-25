#!/bin/bash

# parameter
DBHOST="localhost"
DBPORT="1234"
DBDIR="/home/fabian/mongodb/"
EVALS_MAX="10000"
WORKER_MAX=15


# get current date and time as experiment name
EXPERIMENT=$(date +%Y-%m-%d-%H-%M-%S)
echo "Start experiment $EXPERIMENT ..."

# get the script folder
SCRIPT_FOLDER="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


echo "  Start MongoDB ..."
mongod --dbpath $DBDIR --port $DBPORT &> /dev/null &
sleep 1

echo "  Start kalmanTuning.py ..."
echo "##### START NEW EXPERIMENT $EXPERIMENT #####" >> /tmp/kalmanTuningMain.log
python3 $SCRIPT_FOLDER/kalmanTuning.py --mongohost $DBHOST --mongoport $DBPORT --experiment $EXPERIMENT --max_evals $EVALS_MAX &>> /tmp/kalmanTuningMain.log &
sleep 1

WORKER=0
while [ $WORKER -lt $WORKER_MAX ]
do
  echo "  Start worker $WORKER ..."
  echo "##### START NEW EXPERIMENT $EXPERIMENT #####" >> /tmp/kalmanTuningWorker$WORKER.log
  hyperopt-mongo-worker --mongo=$DBHOST:$DBPORT/$EXPERIMENT --poll-interval=1 --reserve-timeout=10 --max-consecutive-failures=5 &>> /tmp/kalmanTuningWorker$WORKER.log &
  WORKER=$(($WORKER+1))
  sleep 2
done


