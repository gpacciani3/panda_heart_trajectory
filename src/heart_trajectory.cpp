#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

std::vector<geometry_msgs::Pose> generateHeartWaypoints(int num_points = 400) {
    std::vector<geometry_msgs::Pose> waypoints;
    
    for (int i = 0; i < num_points; ++i) {
        double t = 2 * M_PI * i / num_points;
        
        // Parametric equations of the heart in the XZ plane
        double x = 0.1 * std::pow(std::sin(t), 3);
        double z = 0.1 * (13.0 * std::cos(t) - 5.0 * std::cos(2*t) - 2.0 * std::cos(3*t) - std::cos(4*t)) / 16.0;
        
        geometry_msgs::Pose waypoint;
        waypoint.position.x = x + 0.4;  // Offset X
        waypoint.position.y = 0.0;      // FIxed y (XZ plane) 
        waypoint.position.z = z + 0.6;  // Offset Z
        
        // Fixed orientation (points downwards)
        waypoint.orientation.x = 0.0;
        waypoint.orientation.y = 0.0;
        waypoint.orientation.z = 0.0;
        waypoint.orientation.w = 1.0;
        
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

// Function to move the robot to the starting point of the trajectory
bool moveToStartPosition(moveit::planning_interface::MoveGroupInterface& move_group, 
                        const geometry_msgs::Pose& start_pose) {
    move_group.setPoseTarget(start_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    bool success = (move_group.plan(start_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success) {
        ROS_INFO("Successfull planning of the initial movement");
        move_group.execute(start_plan);
        ROS_INFO("Robot placed in the initial point of the heart");
        return true;
    } else {
        ROS_WARN("Failure of planning of the initial movement, attempt of the direct movement...");
        // Attempt of direct movement 
        move_group.move();
        ros::Duration(1.0).sleep();
        return true; 
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "panda_heart_trajectory");
    ros::NodeHandle node_handle;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    // Reference frame
    move_group.setPoseReferenceFrame("panda_link0");  // Robot's base
    move_group.setEndEffectorLink("panda_hand_tcp"); // End-effector link

    move_group.setPlanningTime(30.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.4);

    ROS_INFO("Heart trajectory generation...");
    std::vector<geometry_msgs::Pose> waypoints = generateHeartWaypoints(400);
    
    // The first point is considered as the starting position 
    geometry_msgs::Pose start_pose = waypoints[0];
    
    ROS_INFO("Movement to the starting point of the heart trajectory...");
    if (moveToStartPosition(move_group, start_pose)) {
        ROS_INFO("Starting point reached, start of the heart trajectory...");
        
        // Small time interval after the initial movement
        ros::Duration(1.0).sleep();
        
        moveit_msgs::RobotTrajectory heart_traj;
        moveit_msgs::MoveItErrorCodes error_code;
        double max_eef_step = 0.0005;
        
        // Compute Cartesian Path - exclude the first point (already reached)
        std::vector<geometry_msgs::Pose> trajectory_waypoints(waypoints.begin() + 1, waypoints.end());
        
        double fraction = move_group.computeCartesianPath(
            trajectory_waypoints, 
            max_eef_step, 
            heart_traj, 
            false,
            &error_code
        );
        
        ROS_INFO("Cartesian path completed at %.2f%%", fraction * 100.0);

        if (fraction > 0.8 && error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            moveit::planning_interface::MoveGroupInterface::Plan plan_heart;
            plan_heart.trajectory_ = heart_traj;
            
            ROS_INFO("Heart trajectory execution...");
            move_group.execute(plan_heart);
            
            ROS_INFO("Heart trajectory completed!");
        } else {
            ROS_WARN("Partial path (%.2f%%), but it is executed...", fraction * 100.0);
            if (fraction > 0.4) {
                moveit::planning_interface::MoveGroupInterface::Plan plan_heart;
                plan_heart.trajectory_ = heart_traj;
                move_group.execute(plan_heart);
            } else {
                ROS_ERROR("Too short path for being executed");
            }
        }
    } else {
        ROS_ERROR("Impossible to reach the starting position, trajectory cancelled");
    }

    ros::waitForShutdown();
    return 0;
}

