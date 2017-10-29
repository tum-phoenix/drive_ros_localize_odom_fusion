#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"

ImuOdoOdometry::ImuOdoOdometry(ros::NodeHandle& pnh):
  pnh_(pnh)
{
  imu_sub = new message_filters::Subscriber<drive_ros_msgs::mav_cc16_IMU>(pnh, "/from_mav/imu", 1);
  odo_sub = new message_filters::Subscriber<drive_ros_msgs::mav_cc16_ODOMETER_DELTA>(pnh, "/from_mav/odometer_delta", 1);


  int queue_size = 1;
  policy = new ApproxSyncPolicy(queue_size);
  //policy->setInterMessageLowerBound(0, ros::Duration(0,250000));
  //policy->setMaxIntervalDuration(ros::Duration(0,400000));

  sync = new message_filters::Synchronizer<ApproxSyncPolicy>(static_cast<ApproxSyncPolicy>(*policy), *odo_sub, *imu_sub);
  sync->registerCallback(boost::bind(&ImuOdoOdometry::syncCallback, this, _1, _2));

  time_debug_test = pnh.advertise<drive_ros_msgs::TimeCompare>("/odom/debug_times", 10);
}



ImuOdoOdometry::~ImuOdoOdometry()
{
  delete(sync);
  delete(imu_sub);
  delete(odo_sub);
}


bool ImuOdoOdometry::init()
{

  return true;
}



void ImuOdoOdometry::syncCallback(const drive_ros_msgs::mav_cc16_ODOMETER_DELTAConstPtr &msg_odo,
                                  const drive_ros_msgs::mav_cc16_IMUConstPtr &msg_imu)
{
  ros::Duration diff = msg_imu->header.stamp - msg_odo->header.stamp;
  ROS_INFO_STREAM("msg_imu time:" << msg_imu->header.stamp.toNSec() <<
                  "  msg_odo time:" << msg_odo->header.stamp.toNSec() <<
                  "  diff time:" << diff.toNSec()*pow(10,-3));

  drive_ros_msgs::TimeCompare msg_time;
  msg_time.header.stamp = ros::Time::now();
  msg_time.time_1 = msg_imu->header.stamp;
  msg_time.time_2 = msg_odo->header.stamp;
  msg_time.diff_time = diff;


  time_debug_test.publish(msg_time);

}




//void ImuOdoOdometry::heartbeatCallback(const drive_ros_msgs::mav_cc16_HEARTBEAT::ConstPtr &msg)
//{
//  geometry_msgs::TransformStamped transformStamped;
//  ros::Duration time_diff =  msg->header.stamp - old_msg;
//  double dist, x, y, angle;

//  if(0 != time_diff.toSec())
//    dist = msg->rc_velocity*time_diff.toSec();
//  else
//    dist = 0;

//  geometry_msgs::Quaternion q1 = old_transform.transform.rotation;


//  double roll,yaw,pitch;
//  geometry_msgs::Quaternion q = transformStamped.transform.rotation;
//  tf::Quaternion tfq;
//  tf::quaternionMsgToTF(q1, tfq);
//  tf::Matrix3x3(tfq).getEulerYPR(yaw,pitch,roll);

//  angle =  msg->rc_steering_rear + yaw;

//  x = dist*cos(msg->rc_steering_rear);
//  y = dist*sin(msg->rc_steering_rear);

//  ROS_INFO_STREAM("dis:" << dist << " x:" << x << " y:" << y << " yaw:" << yaw);

//  transformStamped.header.stamp = ros::Time::now();
//  transformStamped.header.frame_id = "odo_simple";
//  transformStamped.child_frame_id = "rear_axis_middle";
//  transformStamped.transform.translation.x = old_transform.transform.translation.x + x;
//  transformStamped.transform.translation.y = old_transform.transform.translation.y + y;
//  transformStamped.transform.translation.z = old_transform.transform.translation.z + 0;
//  tf2::Quaternion q2;
//  q2.setRPY(0, 0, angle);
//  transformStamped.transform.rotation.x =  q2.x();
//  transformStamped.transform.rotation.y =  q2.y();
//  transformStamped.transform.rotation.z =  q2.z();
//  transformStamped.transform.rotation.w =  q2.w();
//  odo_simple.sendTransform(transformStamped);

//  old_transform = transformStamped;
//  old_msg = msg->header.stamp;



//}


