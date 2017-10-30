#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"

// separate thread to run odometry with a different rate
void odoThread(ros::Rate r, ImuOdoOdometry* odom)
{
  while(ros::ok()){
    odom->computeOdometry();
    r.sleep();
  }
}

// main function
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "imu_odo_odometry");
  ros::NodeHandle pnh("~");

  // rate at which the node will run
  int rate;
  pnh.param<int>("rate", rate, 100);
  ros::Rate r(rate);

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  ImuOdoOdometry odom(pnh);
  ROS_INFO("IMU & Odometer odometry node succesfully initialized");


  // start odometry thread
  std::thread odo (odoThread, r, &odom);

  // forever loop
  while(ros::ok()){
    ros::spin();
  }

  return 0;
}
