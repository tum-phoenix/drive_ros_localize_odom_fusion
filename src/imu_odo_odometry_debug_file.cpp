#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"

void ImuOdoOdometry::write_input_header(std::string filename)
{
  file_in_log.open( filename );

  // IMU message
  file_in_log << "imu_timestamp,"
              << "orient_x,"
              << "orient_y,"
              << "orient_z,"
              << "orient_w,";

  for(int i=0; i<9; i++)
    file_in_log << "orient_cov_(" << i << "),";

  file_in_log << "gyro_x,"
              << "gyro_y,"
              << "gyro_z,";

  for(int i=0; i<9; i++)
    file_in_log << "gyro_cov_(" << i << "),";

  file_in_log << "acc_x,"
              << "acc_y,"
              << "acc_z,";

  for(int i=0; i<9; i++)
    file_in_log << "acc_cov_(" << i << "),";

  // odometry message
  file_in_log << "odo_timestamp,"
              << "0_pos_abs,"
              << "0_pos_abs_var,"
              << "0_pos_rel,"
              << "0_pos_rel_var,"
              << "0_vel,"
              << "0_vel_var,"
              << "1_pos_abs,"
              << "1_pos_abs_var,"
              << "1_pos_rel,"
              << "1_pos_rel_var,"
              << "1_vel,"
              << "1_vel_var,"
              << "2_pos_abs,"
              << "2_pos_abs_var,"
              << "2_pos_rel,"
              << "2_pos_rel_var,"
              << "2_vel,"
              << "2_vel_var,"
              << "3_pos_abs,"
              << "3_pos_abs_var,"
              << "3_pos_rel,"
              << "3_pos_rel_var,"
              << "3_vel,"
              << "3_vel_var";


   file_in_log << std::endl;
}


void ImuOdoOdometry::write_output_header(std::string filename)
{
  file_out_log.open( filename );

  file_out_log << "timestamp,";

  file_out_log << "pose_posX,"
               << "pose_posY"
               << "pose_posZ"
               << "pose_oriW"
               << "pose_oriX"
               << "pose_oriY"
               << "pose_oriZ";

  for(int i=0; i<36; i++)
    file_out_log << "pose_cov_[" << i << "],";

  file_out_log << "twist_linX,"
               << "twist_linY,"
               << "twist_linZ,"
               << "twist_angX,"
               << "twist_angY,"
               << "twist_angZ,";

  for(int i=0; i<36; i++)
    file_out_log << "twist_cov_[" << i << "],";

   file_out_log << std::endl;
}


void ImuOdoOdometry::write_input_msgs(const drive_ros_msgs::VehicleEncoderConstPtr &msg_odo,
                                      const sensor_msgs::ImuConstPtr &msg_imu)
{
  // IMU message
  file_in_log << msg_imu->header.stamp.toSec() << ","
              << msg_imu->orientation.x        << ","
              << msg_imu->orientation.y        << ","
              << msg_imu->orientation.z        << ","
              << msg_imu->orientation.w        << ",";

  for(int i=0; i<9; i++)
    file_in_log << msg_imu->orientation_covariance.at(i) << ",";

  file_in_log << msg_imu->angular_velocity.x << ","
              << msg_imu->angular_velocity.y << ","
              << msg_imu->angular_velocity.z << ",";

  for(int i=0; i<9; i++)
    file_in_log << msg_imu->angular_velocity_covariance.at(i) << ",";

  file_in_log << msg_imu->linear_acceleration.x << ","
              << msg_imu->linear_acceleration.y << ","
              << msg_imu->linear_acceleration.z << ",";

  for(int i=0; i<9; i++)
    file_in_log << msg_imu->linear_acceleration_covariance.at(i) << ",";

  // odometer message
  for(int i=0; i< msg_odo->encoder.size(); i++)
  {
    file_in_log << msg_odo->header.stamp.toSec()     << ","
                << msg_odo->encoder[i].pos_abs       << ","
                << msg_odo->encoder[i].pos_abs_var   << ","
                << msg_odo->encoder[i].pos_rel       << ","
                << msg_odo->encoder[i].pos_rel_var   << ","
                << msg_odo->encoder[i].vel           << ","
                << msg_odo->encoder[i].vel_var       << ",";
  }



   file_in_log << std::endl;
}


void ImuOdoOdometry::write_output_result(const nav_msgs::Odometry *msg)
{
  file_out_log << msg->header.stamp.toSec();

  file_out_log << msg->pose.pose.position.x
               << msg->pose.pose.position.y
               << msg->pose.pose.position.z
               << msg->pose.pose.orientation.w
               << msg->pose.pose.orientation.x
               << msg->pose.pose.orientation.y
               << msg->pose.pose.orientation.z;

  for(int i=0; i<36; i++)
    file_out_log << msg->pose.covariance.at(i) << ",";

  file_out_log << msg->twist.twist.linear.x
               << msg->twist.twist.linear.y
               << msg->twist.twist.linear.z
               << msg->twist.twist.angular.x
               << msg->twist.twist.angular.y
               << msg->twist.twist.angular.z;

  for(int i=0; i<36; i++)
    file_out_log << msg->twist.covariance.at(i) << ",";

  file_out_log << std::endl;
}
