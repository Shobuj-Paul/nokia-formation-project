#include <nokia_libraries/AutoPilot.hpp>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

geometry_msgs::Pose AutoPilot::Pose(float x, float y, float z)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        return pose;
    }

void AutoPilot::Mode::setArm(ros::ServiceClient arming_client)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
    }
}

void AutoPilot::Mode::setDisarm(ros::ServiceClient arming_client)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle disarmed");
    }
}

void AutoPilot::Mode::offboard_set_mode(ros::ServiceClient set_mode_client)
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
    }
}

void AutoPilot::Mode::land_set_mode(ros::ServiceClient set_mode_client)
{
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
    {
        ROS_INFO("Landing enabled");
    }
}

void AutoPilot::StateMonitor::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    state = *msg;
}

void AutoPilot::StateMonitor::pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
}

void AutoPilot::StateMonitor::leader_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    leader_pose = *msg;
}

void AutoPilot::StateMonitor::waypoints_list_cb(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    setpoints = *msg;
}

void AutoPilot::StateMonitor::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    velocity = *msg;
}

