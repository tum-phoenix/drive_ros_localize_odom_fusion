#ifndef BASE_MODEL_H
#define BASE_MODEL_H

// system
#include <cmath>
#include <mutex>
#include <thread>
#include <fstream>

// ros
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <tf/tf.h>
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

// covariances
#include "cov_elements.h"


class BaseWrapper
{
public:

  // directly read from bag file
  bool processBag(std::string bag_file_path);

  // init publisher, subscriber and some parameters
  bool initROS(bool use_bag);

  // some typedefs
  typedef drive_ros_msgs::VehicleEncoder VeEnc;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                          sensor_msgs::Imu> SyncPolicy;
protected:

  // initialize Kalman Filter
  virtual bool initFilterState() = 0;
  virtual bool initFilterProcessCov() = 0;

  // collect data and prepare computing
  virtual bool insertMeasurement(const nav_msgs::OdometryConstPtr &odo_msg,
                                 const sensor_msgs::ImuConstPtr &imu_msg) = 0;

  // compute one kalman step
  virtual bool computeFilterStep(const float,
                                 const nav_msgs::OdometryConstPtr &odo_msg,
                                 const sensor_msgs::ImuConstPtr &imu_msg) = 0;

  // publish data
  virtual bool getOutput(geometry_msgs::TransformStamped& tf_msg,
                         nav_msgs::Odometry& odom_msg) = 0;


  // ros node handle
  ros::NodeHandle nh;
  ros::NodeHandle pnh;


 private:
  // callback function for subscriber
  void syncCallback(const nav_msgs::OdometryConstPtr &msg_odo,
                    const sensor_msgs::ImuConstPtr &msg_imu);

  // services
  bool svrReloadProcCov(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);
  bool svrReinitState(std_srvs::Trigger::Request  &req,
                      std_srvs::Trigger::Response &res);

  // debug file operations
  void writeOutputHeader(std::string filename);
  void writeOutputResult(const nav_msgs::Odometry* msg);

  // ROS subscriber + synchronizer
  message_filters::Subscriber<sensor_msgs::Imu> *imu_sub;
  message_filters::Subscriber<nav_msgs::Odometry> *odo_sub;
  message_filters::Synchronizer<SyncPolicy> *sync;
  SyncPolicy* policy;

  // ROS publisher or broadcaster
  tf2_ros::TransformBroadcaster br;
  ros::Publisher odo_pub;

  // services
  ros::ServiceServer reload_proc_cov;
  ros::ServiceServer reinit_state;

  // ROS times and durations
  ros::Time last_timestamp;
  ros::Duration last_delta;

  // parameter
  ros::Duration max_time_between_meas;
  bool use_sensor_time_for_pub;
  std::string static_frame;
  std::string moving_frame;
  std::string odo_topic_name;
  std::string imu_topic_name;

  // debug to file
  bool debug_out_file;
  std::ofstream file_out_log;

};

#endif
