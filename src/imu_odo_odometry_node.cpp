#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"

// main function
int main(int argc, char **argv)
{
  // set up node
  ros::init(argc, argv, "imu_odo_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  // check if we want to read from a bag
  std::string bag_file_path;
  bool use_bag;
  pnh.param<std::string>("bag_file_path", bag_file_path, "/tmp/in.bag");
  pnh.param<bool>("read_from_bag", use_bag, false);

  // initialize ego motion estimator
  ImuOdoOdometry odom(nh, pnh, use_bag);
  ROS_INFO("IMU & Odometer odometry node succesfully initialized");


  if(use_bag){

    // read data directly from a bag
    odom.processBag(bag_file_path);

  }else{

    // spin node normally
    while(ros::ok()){
      ros::spin();
    }
  }


  return 0;
}
