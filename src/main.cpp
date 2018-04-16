#include "drive_ros_localize_odom_fusion/CTRA_wrapper.h"
#include "drive_ros_localize_odom_fusion/CTRV_wrapper.h"

// main function
int main(int argc, char **argv)
{
  // set up node
  ros::init(argc, argv, "odom_fusion");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  // pointer to model
  BaseWrapper* model;

  // check if we want to read from a bag
  std::string bag_file_path;
  bool use_bag;
  pnh.param<std::string>("bag_file_path", bag_file_path, "/tmp/in.bag");
  pnh.param<bool>("read_from_bag", use_bag, false);


  // which model to use?
  std::string vehicle_model;
  pnh.param<std::string>("vehicle_model", vehicle_model, "CTRA");

  if("CTRA" == vehicle_model){

    model = new CTRAWrapper(nh, pnh);

  }else if("CTRV" == vehicle_model){

    model = new CTRVWrapper(nh, pnh);

  }else{

    ROS_ERROR_STREAM("Invalid vehicle model: " << vehicle_model);
    return 1;
  }


  // initialize ros stuff
  if(model->initROS(use_bag))
  {
    ROS_INFO("Odometry fusion node succesfully initialized");
  }else{
    ROS_ERROR("Odometry fusion node failed!");
  }


  // check if read from bag
  if(use_bag){

    // read data directly from a bag
    model->processBag(bag_file_path);

  }else{

    // spin node normally
    while(ros::ok()){
      ros::spin();
    }
  }


  return 0;
}
