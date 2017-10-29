#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "imu_odo_odometry");

  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  ImuOdoOdometry simple_odom(pnh);
  if (!simple_odom.init()) {
    return 1;
  }
  else {
    ROS_INFO("IMU & Odometer odometry node succesfully initialized");
  }

  while (ros::ok()) {
    ros::spin();
}

  return 0;
} // end main()
