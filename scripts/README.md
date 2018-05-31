# Parameter Tuning
This scripts are meant for training the filter parameters using Hyperopt. 

* `runAll.sh`: runs alls trials using multiple workers
* `kalmanTuning.py`: contains the actual objective function and interface to hyperopt
* `runTial.sh`: is invoked by the `kalmanTuning.py` for each trial with different parameters
* `analyzeResults.ipynb`: analyze the finals results
