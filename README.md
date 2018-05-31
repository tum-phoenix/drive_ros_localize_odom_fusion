# Odometry Fusion
Calculates Kalman filtered vehicle odometry based on different sources.

possible inputs for prediction step:
* odometry msg + imu msg (synchronized via message filters)
* single odometry msg
* single imu msg

possible inputs for correction step:
* odometry msg + imu msg (synchronized via message filters)
* single odometry msg
* single imu msg

Correction step rate can be slower than prediction rate.

Currently supported models:
* CTRV
* CTRA

=> please check the docs or code which inputs the model require

## dependencies
- [Kalman Lib](https://github.com/mherb/kalman)

## credit
Some parts are ported from [LMS ego estimator](https://github.com/lms-org/ego_estimator)
- [mherb](https://github.com/orgs/tum-phoenix/people/mherb)
- [Phibedy](https://github.com/orgs/tum-phoenix/people/Phibedy)
- [fabolhak](https://github.com/orgs/tum-phoenix/people/fabolhak)
