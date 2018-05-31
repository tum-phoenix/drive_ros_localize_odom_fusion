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
  if(model->initROS())
  {
    ROS_INFO("Odometry fusion node succesfully initialized");

    // spin node normally
    while(ros::ok()){
      ros::spin();
    }

  }else{
    ROS_ERROR("Odometry fusion node failed!");

    nh.shutdown();
    pnh.shutdown();
  }
  return 0;
}
