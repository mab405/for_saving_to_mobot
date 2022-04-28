#include <ros/ros.h>
#include <mobot_coord/MobotCoord.h> // this message type is defined in the robot_coord package
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <string>
using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "dummy_mobot_coordinator");
    ros::NodeHandle n;
// 3 Services Needed for Steering Mobot, Finding Block, and Grabbing Block

    // Create Perception
    // "perception_service" was original name in given code
    ros::ServiceClient perception_client = n.serviceClient<mobot_coord::MobotCoord>("perception_service"); // GET BLOCK COORDS
    // CHECK IF IT EXISTS
    while (!perception_client.exists()) {
    ROS_INFO("waiting for perception service...");
    ros::Duration(1.0).sleep();
    }   
    ROS_INFO("connected perception service");

    // Create Navigation
    ros::ServiceClient navigation_client = n.serviceClient<mobot_coord::MobotCoord>("navigation_service"); // GO TO STATION COMMANDS
    // CHECK IF IT EXISTS
    while (!navigation_client.exists()) {
    ROS_INFO("waiting for navigation service...");
    ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected navigation service");

    // Create Manipulation
    ros::ServiceClient manipulation_client = n.serviceClient<mobot_coord::MobotCoord>("manipulation_service"); // GRAB AND DROP BLOCK
    // CHECK IF IT EXISTS
    while (!navigation_client.exists()) {
    ROS_INFO("waiting for manipulation service...");
    ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected manipulation service");
 //DONE WITH CREATING STUFF
// ------------------------------------------
//FOR USE WITH TESTING DUMMY SERVICES 
    mobot_coord::MobotCoord mobot_srv;
//END OF DUMMY SERVICE CODE
// ------------------------------------------
//FOR USE WITH REAL ROBOT
    //mobot_coord::MobotCoord manipulation_srv;
    //mobot_coord::MobotCoord perception_srv;
//END OF CODE FOR USE WITH REAL ROBOT
// ------------------------------------------
    ROS_INFO("attempting to send request to navigation service to go to station 1");
    mobot_srv.request.request_code= mobot_coord::MobotCoordRequest::GO_TO_STATION_1;
    navigation_client.call(mobot_srv);
    int response = mobot_srv.response.response_code;
    if (!response) {ROS_WARN("navigator responded with failure!"); }
    else {ROS_INFO("navigator responded with success");}
    
    
    ROS_INFO("attempting request to perception service: ");
    mobot_srv.request.request_code= mobot_coord::MobotCoordRequest::GET_BLOCK_COORDS; //not really necessary; perception service can assume this request
    double x,y,z;

   if (perception_client.call(mobot_srv)) {
   	x = mobot_srv.response.perceived_block_pose.pose.position.x;
   	y = mobot_srv.response.perceived_block_pose.pose.position.y;
   	z = mobot_srv.response.perceived_block_pose.pose.position.z;   	
   	ROS_INFO("perception service returned values x,y,z = %f, %f, %f",x,y,z);

        } else {
            ROS_ERROR("Failed to call service perception_service");
            return 1; //crash and burn...could use better error handling!
        }
    response = mobot_srv.response.response_code;
    if (!response) {ROS_WARN("perception responded with failure!"); }
    else {ROS_INFO("perception responded with success"); }     
        
        
    ROS_INFO("attempting request to manipulation service to grab the block");
        mobot_srv.request.request_code = mobot_coord::MobotCoordRequest::GRAB_BLOCK; 
        mobot_srv.request.grasp_block_pose = mobot_srv.response.perceived_block_pose; //just copy over the perceived block coords as grasp coords
        manipulation_client.call(mobot_srv);

    // Assuming it gets to this point successfully, its now holding the block at Table 1
    ROS_INFO("attempting to send request to navigation service to go to station 2");
    mobot_srv.request.request_code= mobot_coord::MobotCoordRequest::GO_TO_STATION_2;
    navigation_client.call(mobot_srv);
    if (!response) {ROS_WARN("navigator responded with failure!"); }
    else {ROS_INFO("navigator responded with success");}

    // Assuming Success its now at Table 2 and needs to drop the block
    ROS_INFO("attempting request to manipulation service to grab the block");
        mobot_srv.request.request_code = mobot_coord::MobotCoordRequest::DROP_BLOCK; 
        mobot_srv.request.grasp_block_pose = mobot_srv.response.perceived_block_pose; //just copy over the perceived block coords as grasp coords
        manipulation_client.call(mobot_srv);


    // Now that it has successfully dropped bolo9ck we go back to Home
    ROS_INFO("attempting to send request to navigation service to go to Home");
    mobot_srv.request.request_code= mobot_coord::MobotCoordRequest::GO_HOME;
    navigation_client.call(mobot_srv);
    if (!response) {ROS_WARN("navigator responded with failure!"); }
    else {ROS_INFO("navigator responded with success");}
    
    // Now it is good and done
    return 0;
}
