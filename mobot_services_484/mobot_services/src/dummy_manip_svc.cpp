//shell for manipulation service
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
    double x,y,z;
    if (request.request_code==mobot_coord::MobotCoordRequest::DROP_BLOCK) {
      ROS_INFO("I have been requested to drop the block; I will run a hard-coded routine to do so");
      response.response_code = mobot_coord::MobotCoordResponse::SUCCESS; 
        return true;
    }
    else if (request.request_code==mobot_coord::MobotCoordRequest::GRAB_BLOCK) {
      x = request.grasp_block_pose.pose.position.x;
      y = request.grasp_block_pose.pose.position.y;
      z = request.grasp_block_pose.pose.position.z;      
      ROS_INFO("I have been requested to grab a block from coordinates x,y,z = %f, %f, %f",x,y,z);
      // so do it...
      
      response.response_code = mobot_coord::MobotCoordResponse::SUCCESS; 
        return true;
    }
      else {
        ROS_WARN("manipulation code not recognized!");
        response.response_code = mobot_coord::MobotCoordResponse::FAILURE;
      }




  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_manip_svc");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("manipulation_service", callback);
  ROS_INFO("ready to accept manipulation requests");
  ros::spin();

  return 0;
}


