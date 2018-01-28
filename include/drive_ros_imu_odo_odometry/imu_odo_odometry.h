#ifndef IMUODOODOMETRY_H
#define IMUODOODOMETRY_H

// system
#include <cmath>
#include <mutex>
#include <thread>
#include <fstream>

// ros
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ros messages
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "drive_ros_msgs/VehicleEncoder.h"
#include "drive_ros_msgs/TimeCompare.h"

// ros services
#include <std_srvs/Trigger.h>

// kalman
#include <kalman/ExtendedKalmanFilter.hpp>
#include "measurement_model.h"
#include "system_model.h"
#include "cov_elements.h"



class ImuOdoOdometry
{
public:
  //! Constructor.
  ImuOdoOdometry(ros::NodeHandle& nh, ros::NodeHandle& pnh, ros::Rate& r);

  //! Destructor.
  ~ImuOdoOdometry();

  void computeOdometry();

  typedef float T;

  typedef drive_ros_msgs::VehicleEncoder VeEnc;

  typedef CTRA::State<T> State;
  typedef CTRA::Control<T> Control;
  typedef CTRA::Measurement<T> Measurement;

  typedef CTRA::MeasurementModel<T> MeasurementModel;
  typedef CTRA::SystemModel<T> SystemModel;
  typedef Kalman::ExtendedKalmanFilter<State> Filter;

  typedef message_filters::sync_policies::ApproximateTime<drive_ros_msgs::VehicleEncoder,
                                                          sensor_msgs::Imu> SyncPolicy;

  // inject messages directly (only for debug)
  void set_imu_msg(const sensor_msgs::Imu &msg){mut.lock(); imu_msg = msg; mut.unlock();};
  void set_odo_msg(const drive_ros_msgs::VehicleEncoder &msg){mut.lock(); odo_msg = msg; mut.unlock();};

private:

  // initialize Kalman Filter
  void initFilterState();
  void initFilterProcessCov();


  // collect data and prepare computing
  bool computeMeasurement(const drive_ros_msgs::VehicleEncoder &odo_msg,
                          const sensor_msgs::Imu &imu_msg);

  // compute one kalman step
  bool computeFilterStep();

  // publish data
  bool publishCarState();


  // Callback function for subscriber
  void syncCallback(const drive_ros_msgs::VehicleEncoderConstPtr &msg_odo,
                    const sensor_msgs::ImuConstPtr &msg_imu);


  // services
  bool svr_reload_proc_cov(std_srvs::Trigger::Request  &req,
                           std_srvs::Trigger::Response &res);
  bool svr_reinit_state(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);


  // debug file operations
  void write_input_header(std::string filename);
  void write_input_msgs(const drive_ros_msgs::VehicleEncoderConstPtr &msg_odo,
                        const sensor_msgs::ImuConstPtr &msg_imu);

  void write_output_header(std::string filename);
  void write_output_result(const nav_msgs::Odometry* msg);

  // ROS subscriber + synchronizer
  message_filters::Subscriber<sensor_msgs::Imu> *imu_sub;
  message_filters::Subscriber<drive_ros_msgs::VehicleEncoder> *odo_sub;
  message_filters::Synchronizer<SyncPolicy> *sync;
  SyncPolicy* policy;

  // ROS publisher or broadcaster
  tf2_ros::TransformBroadcaster br;
  ros::Publisher odo_pub;

  ros::Rate rate;
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  // services
  ros::ServiceServer reload_proc_cov;
  ros::ServiceServer reinit_state;

  // ROS local message storage + mutex
  drive_ros_msgs::VehicleEncoder odo_msg;
  sensor_msgs::Imu imu_msg;
  std::mutex mut;

  // kalman filter stuff
  Control u;
  Measurement z;
  SystemModel sys;
  MeasurementModel mm;
  Filter filter;

  // ROS times and durations
  ros::Time lastTimestamp;
  ros::Time currentTimestamp;
  ros::Duration currentDelta;

  // parameter
  ros::Duration max_time_between_meas;
  bool ignore_acc_values;
  bool use_sensor_time_for_pub;
  std::string static_frame;
  std::string moving_frame;

  // debug to file
  bool debug_out_file;
  std::ofstream file_out_log;

  bool debug_in_file;
  std::ofstream file_in_log;

};

#endif
