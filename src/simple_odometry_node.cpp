#include "drive_ros_imu_odo_odometry/simple_odometry.h"


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "talker");

  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  SimpleOdometry simple_odom(pnh);
  if (!simple_odom.init()) {
    return 1;
  }
  else {
    ROS_INFO("Simple odometry node succesfully initialized");
  }

  while (ros::ok()) {
    ros::spin();
}

  return 0;
} // end main()
