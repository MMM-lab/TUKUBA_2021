/*
 * ********************************************************************************
 * SYSTEM            | actionlib multiple goal specification
 * ********************************************************************************
*/
/***********************************************************************
 * Header files 
 **********************************************************************/
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/***********************************************************************
 * Global variables
 **********************************************************************/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
/*
* ***********************************************************************
* x   | x-coordinate
* y   | y-coordinate
* yaw | yaw-coordinate
* ***********************************************************************
*/
struct robot_pose {
    double x;
    double y;
    double yaw;
};

/**
 *****************************************************************************************************
 * main loop
 *****************************************************************************************************
 */
int main(int argc, char** argv){
    /***********************************************************************
     * multiple goal(way_point)
     **********************************************************************/
    robot_pose way_point[] = {{-0.15, -0.54, 0.0 * M_PI}, {1.8, -0.5, M_PI}, {1.6, 1.5,  0.0 * M_PI}, {999, 999, 999}};
    //robot_pose way_point[] = {{0.0, 1.0, 0.5 * M_PI}, {-1.0, 1.0, M_PI}, {999, 999, 999}};

    ros::init(argc, argv, "multiple_goal_specification");
    ros::NodeHandle nh;
    ros::Publisher pub= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);

    // Setting the Initial Position turtlebot3_worldの初期位置が(-2.0, -0.5)なのでそれに設定
    geometry_msgs::PoseWithCovarianceStamped initialpose;
    initialpose.header.stamp            = ros::Time::now(); 
    initialpose.header.frame_id         = "map";  
    initialpose.pose.pose.position.x    = -2.0;
    initialpose.pose.pose.position.y    = -0.5;
    initialpose.pose.pose.position.z    = 0.0;
    initialpose.pose.pose.orientation.w = 1.0;
    // Send the initial position to the amcl package
    pub.publish(initialpose);

    /*
    * *****************************************************************************
    * - Create an action client
    * - where the first argument is the name of the action server to connect to
    * - The action server has to be up and running
    * - The second argument, if true, spins the thread automatically (ros::spin())
    * *****************************************************************************
    */
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("The server comes up");
    move_base_msgs::MoveBaseGoal goal;

    int goal_index = 0;
    while (ros::ok()) {
        goal.target_pose.header.stamp     = ros::Time::now();
        goal.target_pose.header.frame_id  = "base_footprint";     
        goal.target_pose.pose.position.x  =  way_point[goal_index].x;
        goal.target_pose.pose.position.y  =  way_point[goal_index].y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[goal_index].yaw);

        if (goal.target_pose.pose.position.x == 999) {
            break;
        }

        ROS_INFO("Sending goal: No.%d", goal_index + 1);
        // Send goal to the server
        ac.sendGoal(goal);

        // Wait 30.0[s] for the results to come back. This is where it is blocked.
        bool succeeded = ac.waitForResult(ros::Duration(30.0));

        // Look at the results and mark Succeeded for success or Failed for failure
        actionlib::SimpleClientGoalState state = ac.getState();

        if(succeeded) {
            ROS_INFO("Succeeded: No.%d (%s)",goal_index + 1, state.toString().c_str());
        }
        else {
            ROS_INFO("Failed: No.%d (%s)",goal_index + 1, state.toString().c_str());
        }
        goal_index++;  
    }
    return 0;
}