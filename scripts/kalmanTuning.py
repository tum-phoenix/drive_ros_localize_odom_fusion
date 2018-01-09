import csv
import argparse
from hyperopt import fmin, tpe, hp
from hyperopt.mongoexp import MongoTrials

# paths
path_results =      "/data/KalmanTuningLogs/"



## objective function
# for each trial this function is called and starts the trial runner script
def objective(params):
    # if we run parallel workers then only the objective function is called
    # thats why we can not use anything defined outside of this function :(
    import os
    import csv
    import yaml
    import math
    import time
    import random
    import subprocess
    from hyperopt import STATUS_OK, STATUS_FAIL
    from collections import deque

    path_home = "/home/fabian"
    path_catkin_ws = path_home + "/catkin_ws"
    path_ros_package = path_catkin_ws + "/src/drive_ros_config/modules/drive_ros_imu_odo_odometry"
    path_config_file = path_ros_package + "/config/phoenix_cc2017.yaml"
    path_trial_runner = path_ros_package + "/scripts/runTrial.sh"
    path_bag_file = path_ros_package + "/data/circle.bag"
    path_results = "/data/KalmanTuningLogs/"

    # get trial number
    trial = random.getrandbits(128)

    # check if trial number already exists
    while os.path.exists(path_results + str(trial)):
        trial = random.getrandbits(128)

    # create directory for this trial
    os.makedirs(path_results + str(trial))

    # config file path
    yaml_config = path_results + str(trial) + "/config.yaml"

    # load config
    config_file_in = path_config_file
    config_in = open(config_file_in, 'r')
    config = yaml.load(config_in)

    # update config
    config['kalman_cov']['sys_var_x'] = params['x']
    config['kalman_cov']['sys_var_y'] = params['y']
    config['kalman_cov']['sys_var_a'] = params['a']
    config['kalman_cov']['sys_var_v'] = params['v']
    config['kalman_cov']['sys_var_theta'] = params['theta']
    config['kalman_cov']['sys_var_omega'] = params['omega']

    # save config
    config_out = open(yaml_config, 'w')
    yaml.dump(config, config_out)

    # start trial runner
    p = subprocess.run([path_trial_runner, "--trail", str(trial),
                        "--logdir", path_results + str(trial),
                        "--bag", path_bag_file,
                        "--config", yaml_config,
                        "--catkin_ws", path_catkin_ws], shell=False)

    # evaluate return code of trial runner
    if 0 == p.returncode:
        status = STATUS_OK
    else:
        status = STATUS_FAIL


    # calculate loss
    # open csv file
    with open(path_results + str(trial) + "/odom.csv", 'r') as f:
        try:
            lastrow = deque(csv.reader(f), 1)[0]
        except IndexError:  # empty file
            lastrow = None

        x = float(lastrow[2])  # x position at the end
        y = float(lastrow[3])  # y position at the end

        loss = math.sqrt(x * x + y * y)  # distance from start to the end (assume we drive a circle)

    # return trial results
    return {
        'loss': loss,
        'status': status,
        'eval_time': time.time(),
        'log_path': path_results + str(trial)
    }


## main function
if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Main Kalman Tuning script. Creates all trials and connects to the MongoDB where results are stored.')
    parser.add_argument('--mongohost', help='database hostname or ip', required=True)
    parser.add_argument('--mongoport', help='database port', type=int, required=True)
    parser.add_argument('--experiment', help='name of experiment', required=True)
    parser.add_argument('--max_evals', help='how many evaluations should be tried', type=int, required=True)
    args = parser.parse_args()

    # create search space
    space = {
        'x': hp.uniform('x', 0, 1),
        'y': hp.uniform('y', 0, 1),
        'a': hp.uniform('a', 0, 1),
        'v': hp.uniform('v', 0, 1),
        'theta': hp.uniform('theta', 0, 1),
        'omega': hp.uniform('omega', 0, 1)
    }

    # some settings
    algo = tpe.suggest
    max_evals = args.max_evals
    mongo_host = args.mongohost
    mongo_port = args.mongoport
    mongo_db = args.experiment

    trials = MongoTrials("mongo://" + mongo_host + ":" + str(mongo_port) + "/" + mongo_db + "/jobs",
                         exp_key="kalman")


    # find the hyperparameters
    best = fmin(objective, space, algo, max_evals, trials)

    # save results to file
    with open(path_results + args.experiment + "_results.csv", 'w') as csvfile:
        fieldnames = ['eval_time', 'status', 'loss', 'log_path']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for result in trials.results:
            writer.writerow(result)
