#ifndef IMUODOODOMETRY_H
#define IMUODOODOMETRY_H

#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>

#include "drive_ros_msgs/mav_cc16_IMU.h"
#include "drive_ros_msgs/mav_cc16_ODOMETER_DELTA.h"
#include "drive_ros_msgs/TimeCompare.h"

#include <kalman/UnscentedKalmanFilter.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
#include "measurement_model.h"
#include "system_model.h"

#define GRAVITY T(9.81)


class ImuOdoOdometry
{
public:
  //! Constructor.
  ImuOdoOdometry(ros::NodeHandle& pnh);

  //! Destructor.
  ~ImuOdoOdometry();

  void computeOdometry();

  typedef float T;

  typedef CTRA::State<T> State;
  typedef CTRA::Control<T> Control;
  typedef CTRA::Measurement<T> Measurement;

  typedef CTRA::MeasurementModel<T> MeasurementModel;
  typedef CTRA::SystemModel<T> SystemModel;
  typedef Kalman::ExtendedKalmanFilter<State> Filter;

  typedef message_filters::sync_policies::ExactTime<drive_ros_msgs::mav_cc16_ODOMETER_DELTA,
                                                    drive_ros_msgs::mav_cc16_IMU> SyncPolicy;


private:

  void computeMeasurement(const drive_ros_msgs::mav_cc16_ODOMETER_DELTA &odo_msg,
                          const drive_ros_msgs::mav_cc16_IMU &imu_msg);
  void computeFilterStep();
  void publishCarState();


  //! Callback function for subscriber.
  void syncCallback(const drive_ros_msgs::mav_cc16_ODOMETER_DELTAConstPtr &msg_odo,
                    const drive_ros_msgs::mav_cc16_IMUConstPtr &msg_imu);

  drive_ros_msgs::mav_cc16_ODOMETER_DELTA odo_msg;
  drive_ros_msgs::mav_cc16_IMU imu_msg;

  std::mutex mut;


  message_filters::Subscriber<drive_ros_msgs::mav_cc16_IMU> *imu_sub;
  message_filters::Subscriber<drive_ros_msgs::mav_cc16_ODOMETER_DELTA> *odo_sub;
  message_filters::Synchronizer<SyncPolicy> *sync;
  SyncPolicy* policy;

  tf2_ros::TransformBroadcaster br;
  ros::Publisher vis_pub;

  ros::NodeHandle pnh_;

  Control u;
  Measurement z;

  SystemModel sys;
  MeasurementModel mm;

  Filter filter;

  ros::Time lastTimestamp;
  ros::Time currentTimestamp;
  ros::Duration previousDelta;

  bool debug_rviz;
  std::string tf_parent;
  std::string tf_child;
  int ct;

  double odometer_velo_cov_xx;
  double imu_acc_cov_xx;
  double imu_acc_cov_yy;
  double imu_gyro_cov_zz;
  double imu_acc_bias_x;
  double imu_acc_bias_y;
  double imu_acc_bias_z;
  double imu_gyro_bias_x;
  double imu_gyro_bias_y;
  double imu_gyro_bias_z;



};

#endif
