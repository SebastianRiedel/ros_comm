//
// Created by sebastian on 06.02.17.
//

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "ros/console.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setboolclient");
  ros::NodeHandle n;

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/setbool");
  ros::Duration(2.).sleep();
  std_srvs::SetBool call_params;
  call_params.request.data = true;
  client.call(call_params);

  ROS_INFO_STREAM("Sent Request:\n" << call_params.request);
  ROS_INFO_STREAM("--\nReceived Response:\n" << call_params.response);

  return 0;
}
