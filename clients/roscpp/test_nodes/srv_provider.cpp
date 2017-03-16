//
// Created by sebastian on 06.02.17.
//

#include "ros/ros.h"
#include "std_srvs/SetBool.h"

bool setbool(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res)
{
    ROS_INFO("Setting bool to request: %d", req.data);
    res.message = "Done.";
    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "setboolserver");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("setbool", setbool);
    ROS_INFO("Ready to set bool.");
    ros::spin();

    return 0;
}