//shell for navigation service
// this is compatible with dummy_robot_coordinator
// start this node before running dummy_robot_coordinator



#include <ros/ros.h>
#include <mobot_coord/MobotCoord.h> // this message type is defined in the current package
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <string>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <des_pub_state_service/ServiceMsg.h>
#include <traj_builder/traj_builder.h>

using namespace std;


nav_msgs::Odometry current_state;
geometry_msgs::PoseStamped current_pose;

ros::ServiceClient client;

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    // ROS_INFO("/current state",current_state);
    current_pose.pose = current_state.pose.pose;
}

bool move2coord(float goal_pose_x, float goal_pose_y)
{
    bool success = true;
    TrajBuilder trajBuilder;
    des_pub_state_service::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose_trans;
    geometry_msgs::PoseStamped goal_pose_rot;
    string mode;
    start_pose.pose = current_state.pose.pose;

    bool success_rotate;
    bool success_translate;

    // For now: rotate to head forward to goal point, then move toward the place.
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = goal_pose_x;
    double y_end = goal_pose_y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;

    double des_psi = atan2(dy, dx);

    ROS_INFO("Start_x = = %f", x_start);
    ROS_INFO("Start_y = = %f", y_start);
    ROS_INFO("Goal_x = %f", x_end);
    ROS_INFO("Goal_y = %f", y_end);
    // rotate
    goal_pose_rot = trajBuilder.xyPsi2PoseStamped(current_pose.pose.position.x,
                                                  current_pose.pose.position.y,
                                                  des_psi); // keep the same x,y, only rotate to des_psi
    srv.request.start_pos = current_pose;
    srv.request.goal_pos = goal_pose_rot;
    srv.request.mode = "2"; // spin so that head toward the goal.
    if (client.call(srv))
    {
        success_rotate = srv.response.success;
        ROS_INFO("rotate success? %d", success_rotate);
    }
    ros::spinOnce();

    // forward
    goal_pose_trans = trajBuilder.xyPsi2PoseStamped(goal_pose_x,
                                                    goal_pose_y,
                                                    des_psi); // keep des_psi, change x,y
    srv.request.start_pos = goal_pose_rot;
    srv.request.goal_pos = goal_pose_trans;
    srv.request.mode = "1"; // spin so that head toward the goal.
    if (client.call(srv))
    {
        success_translate = srv.response.success;
        ROS_INFO("translate success? %d", success_translate);
    }
    ros::spinOnce();

    // if fail to forward
    if (!success_translate)
    {
        ROS_INFO("Cannot move, obstacle. braking");
        srv.request.start_pos = current_pose;
        srv.request.goal_pos = current_pose; //anything is fine.
        srv.request.mode = "3";              // spin so that head toward the goal.
        client.call(srv);
        success = false;
    }
    ros::spinOnce();

    return success;
}

void tryMove(float goal_pose_x, float goal_pose_y, int retry_max)
{
    int retry_ctr = 0;
    bool success = move2coord(goal_pose_x, goal_pose_y);
    while (!success && retry_ctr < retry_max) {
        ROS_WARN("RETRY %d", retry_ctr);
        retry_ctr++;
        success = move2coord(goal_pose_x,goal_pose_y);
    }
}

void backUp()
{
    ROS_INFO("Backing up");
    TrajBuilder trajBuilder;
    des_pub_state_service::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;

    start_pose.pose = current_state.pose.pose;

    srv.request.start_pos = current_pose;
    srv.request.goal_pos = current_pose;
    srv.request.mode = "4"; 
    if (client.call(srv))
    {
        bool success_backup = srv.response.success;
        ROS_INFO("rotate success? %d", success_backup);
    }
    ros::spinOnce();
}



bool callback(mobot_coord::MobotCoordRequest& request, mobot_coord::MobotCoordResponse& response)
{
    ROS_INFO("callback activated");
    int destination_code = request.request_code;
    if (destination_code==mobot_coord::MobotCoordRequest::GO_TO_STATION_1) {
      ROS_INFO("I have been instructed to navigate to station 1"); 
      //so...insert code here to do it!

              float x_1 = 1.216;
              float y_1 = 0.500;

              float x_2 = 2.438;
              float y_2 = 0.500;

              float x_3 = 3.848;
              float y_3 = 0.500;

              ROS_INFO("STEP 1");
              tryMove(x_1, y_1, 1);

              ROS_INFO("STEP 2");
              tryMove(x_2, y_2, 1);

              ROS_INFO("STEP 3");
              tryMove(x_3, y_3, 0);

          ros::Duration(2.0).sleep();  //dummy wait time to look like robot is working
          response.response_code = mobot_coord::MobotCoordResponse::SUCCESS;
          ROS_INFO("done");
          }
    else if (destination_code==mobot_coord::MobotCoordRequest::GO_TO_STATION_2) {
      ROS_INFO("I have been instructed to navigate to station 2"); 
      //ditto 

            float x_4 = 0.432;
            float y_4 = 0.417;

            float x_5 = 0.432;
            float y_5 = 1.447;

            float x_6 = 0.432;
            float y_6 = 2.555;


            backUp();
    
            ROS_INFO("STEP 4");
            tryMove(x_4, y_4, 0);
            

            ROS_INFO("STEP 5");
            tryMove(x_5, y_5, 0);

            ROS_INFO("STEP 6");
            tryMove(x_6, y_6, 0);
            
            backUp();

          ros::Duration(2.0).sleep();  //dummy wait time to look like robot is working
          response.response_code = mobot_coord::MobotCoordResponse::SUCCESS;  
          ROS_INFO("done");    
      }
    else if (destination_code==mobot_coord::MobotCoordRequest::GO_HOME) {      
      ROS_INFO("I have been instructed to navigate to home"); 
      //so do it... 
          float x_o = 0;
          float y_o = 0;

          ROS_INFO("STEP 7");
          tryMove(x_o, y_o, 0);

          ros::Duration(2.0).sleep();  //dummy wait time to look like robot is working
          response.response_code = mobot_coord::MobotCoordResponse::SUCCESS;    
          ROS_INFO("done");  
      }
      else {
        ROS_WARN("navigation code not recognized!");
        response.response_code = mobot_coord::MobotCoordResponse::FAILURE;
      }




  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_nav_svc");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("navigation_service", callback);
  ROS_INFO("ready to accept navigation requests");


    ros::init(argc, argv, "navigation_coordinator");

    vector<geometry_msgs::PoseStamped> plan_points;

    client = n.serviceClient<des_pub_state_service::ServiceMsg>("des_state_publisher_service");

    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    TrajBuilder trajBuilder;

  ros::spin();

  return 0;



  
}


