#pragma once

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

namespace AutoPilot
{
    class Mode
    {   
        public:
        void setArm(ros::ServiceClient arming_client); //Arming Function

        void setDisarm(ros::ServiceClient arming_client); //Disarming Function

        void offboard_set_mode(ros::ServiceClient set_mode_client); //Offboard Mode Setting Function

        void land_set_mode(ros::ServiceClient set_mode_client); //Land Mode Setting Function
    };

    class StateMonitor //For state feedbacks
    {
        public:
        mavros_msgs::State state;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseStamped leader_pose;
        geometry_msgs::PoseArray setpoints;

        void state_cb(const mavros_msgs::State::ConstPtr& msg); //state feedback

        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); //position feedback

        void leader_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); //leader position feedback

        void waypoints_list_cb(const geometry_msgs::PoseArray::ConstPtr& msg); //waypoints list feedback
    };
}
