//shell for perception service
// this is compatible with dummy_robot_coordinator
// start this node before running dummy_robot_coordinator



#include <ros/ros.h>
#include <mobot_coord/MobotCoord.h> // this message type is defined in the mobot_coord package
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <string>
using namespace std;


bool callback(mobot_coord::MobotCoordRequest& request, mobot_coord::MobotCoordResponse& response)
{
    ROS_INFO("callback activated");
    double x=1.0;
    double y= 2.0; 
    double z=3.0;
    ROS_INFO("sending values: x,y,z = %f, %f, %f",x,y,z);
    geometry_msgs::PoseStamped block_coords;
    block_coords.pose.position.x=x;
    block_coords.pose.position.y=y;
    block_coords.pose.position.z=z;
    response.perceived_block_pose=block_coords;
    response.response_code = mobot_coord::MobotCoordResponse::SUCCESS;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_percep_svc");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("perception_service", callback);
  ROS_INFO("ready to accept perception requests");
  ros::spin();

  return 0;
}


