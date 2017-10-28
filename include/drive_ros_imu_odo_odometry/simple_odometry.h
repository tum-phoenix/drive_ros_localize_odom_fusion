#ifndef SIMPLE_ODOMETRY_H
#define SIMPLE_ODOMETRY_H

#include "ros/ros.h"
#include "drive_ros_mavlink_cc2016/HEARTBEAT.h"

class SimpleOdometry
{
public:
  //! Constructor.
  SimpleOdometry(const ros::NodeHandle& pnh);

  //! Destructor.
  ~SimpleOdometry();

  //! Initializer
  bool init();

private:
  //! Callback function for subscriber.
  void messageCallback(const drive_ros_mavlink_cc2016::HEARTBEAT::ConstPtr &msg);

  ros::Subscriber heatbeat_sub;


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

};

#endif // SIMPLE_ODOMETRY_H
