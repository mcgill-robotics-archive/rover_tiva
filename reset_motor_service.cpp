#include <stdbool.h>
#include <stdint.h>
#include <string>



#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include "rover_tiva/ResetMotor.h"



std_msgs::Bool reset_msg;
ros::Publisher reset_publisher;

bool reset_cb(const rover_tiva::ResetMotorRequest& request, rover_tiva::ResetMotorResponse& response){
  ROS_DEBUG("Sending initial message");
  ros::NodeHandle nh_temp;

  reset_msg.data = true;
  reset_publisher = nh_temp.advertise<std_msgs::Bool>(request.MotorName, 1);
  reset_publisher.publish(reset_msg);

  ros::Duration(0.5).sleep();

  ROS_DEBUG("disabling reset message");
  reset_msg.data = false;
  reset_publisher.publish(reset_msg);

  // delete reset_publisher;
  response.Success = true;
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_reset_server");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService<rover_tiva::ResetMotorRequest, rover_tiva::ResetMotorResponse>("motor_reset", &reset_cb);
  ros::spin();
}
