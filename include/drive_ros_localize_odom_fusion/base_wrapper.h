#ifndef BASE_MODEL_H
#define BASE_MODEL_H

// system
#include <cmath>
#include <mutex>
#include <fstream>

// ros
#include <ros/ros.h>
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

// ros services
#include <std_srvs/Trigger.h>

// covariances
#include "cov_elements.h"


class BaseWrapper
{
public:

  // init publisher, subscriber and some parameters
  bool initROS();

  // some typedefs
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                          sensor_msgs::Imu> SyncPolicy;
protected:

  // initialize Kalman Filter
  virtual bool initFilterState() = 0;


  // Kalman filter prediction
  virtual bool predict(const float dt,
                       const nav_msgs::OdometryConstPtr &odo_msg,
                       const sensor_msgs::ImuConstPtr &imu_msg) = 0;

  // Kalman filter correction
  virtual bool correct(const float dt,
                       const nav_msgs::OdometryConstPtr &odo_msg,
                       const sensor_msgs::ImuConstPtr &imu_msg) = 0;

  // publish data
  virtual bool getOutput(geometry_msgs::TransformStamped& tf_msg,
                         nav_msgs::Odometry& odom_msg) = 0;


  // ros node handle
  ros::NodeHandle nh;
  ros::NodeHandle pnh;


 private:
  // reset filter and times
  bool reset();

  // calculates the current/old timestamp/delta
  bool processTimestamp(ros::Time& last_t, ros::Time& curr_t,
                        ros::Duration& last_d, ros::Duration& curr_d) const;

  // services
  bool svrReset(std_srvs::Trigger::Request  &req,
                std_srvs::Trigger::Response &res);

  // PREDICTION: process the prediction data
  bool processPredictionData(ros::Time current_timestamp,
                             const nav_msgs::OdometryConstPtr &msg_odo,
                             const sensor_msgs::ImuConstPtr &msg_imu);

  // CORRECTION: process the correction data
  bool processCorrectionData(ros::Time current_timestamp,
                             const nav_msgs::OdometryConstPtr &msg_odo,
                             const sensor_msgs::ImuConstPtr &msg_imu);


  // PREDICTION callback functions
  void predSyncCallback(const nav_msgs::OdometryConstPtr &msg_odo,  // both available
                        const sensor_msgs::ImuConstPtr &msg_imu);
  void predOdoCallback (const nav_msgs::OdometryConstPtr &msg_odo); // only odo available
  void predImuCallback (const sensor_msgs::ImuConstPtr &msg_imu);   // only imu available

  // CORRECTION callback functions
  void corrSyncCallback(const nav_msgs::OdometryConstPtr &msg_odo,  // both available
                        const sensor_msgs::ImuConstPtr &msg_imu);
  void corrOdoCallback (const nav_msgs::OdometryConstPtr &msg_odo); // only odo available
  void corrImuCallback (const sensor_msgs::ImuConstPtr &msg_imu);   // only imu available


  // PREDICTION subscriber and message filter stuff
  message_filters::Subscriber<sensor_msgs::Imu>   *pred_imu_sub;
  message_filters::Subscriber<nav_msgs::Odometry> *pred_odo_sub;
  message_filters::Synchronizer<SyncPolicy>       *pred_sync;
  SyncPolicy                                      *pred_policy;

  // PREDICTION single subscriber
  ros::Subscriber pred_imu_single_sub;
  ros::Subscriber pred_odo_single_sub;

  // CORRECTION subscriber and message filter stuffe
  message_filters::Subscriber<sensor_msgs::Imu>   *corr_imu_sub;
  message_filters::Subscriber<nav_msgs::Odometry> *corr_odo_sub;
  message_filters::Synchronizer<SyncPolicy>       *corr_sync;
  SyncPolicy                                      *corr_policy;

  // CORRECTION single subscriber
  ros::Subscriber corr_imu_single_sub;
  ros::Subscriber corr_odo_single_sub;


  // ROS times and durations
  ros::Time     pred_last_timestamp;
  ros::Duration pred_last_delta;
  ros::Time     corr_last_timestamp;
  ros::Duration corr_last_delta;

  // ROS publisher or tf broadcaster
  tf2_ros::TransformBroadcaster br;
  ros::Publisher odo_pub;

  // services
  ros::ServiceServer reload_proc_cov;

  // model mutex
  std::mutex model_mutex;
  bool predict_since_last_correct;

  // parameter
  ros::Duration time_threshold;
  std::string static_frame;
  std::string moving_frame;
  std::string corr_odo_topic;
  std::string pred_odo_topic;
  std::string corr_imu_topic;
  std::string pred_imu_topic;

  // debug to file
  bool debug_out_file;
  std::ofstream file_out_log;

};

#endif
