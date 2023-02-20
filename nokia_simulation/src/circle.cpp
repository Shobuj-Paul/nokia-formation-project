#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nokia_libraries/AutoPilot.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "setpoint_test");
    ros::NodeHandle node;
    AutoPilot::StateMonitor *state = new AutoPilot::StateMonitor;
    AutoPilot::Mode mode;
    ros::Rate rate(20);
 
    //Subscribe to state feedbacks
    ros::Subscriber state_sub = node.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, &AutoPilot::StateMonitor::state_cb, state); 
    //Subscribe to position feedbacks
    ros::Subscriber pos_sub = node.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, &AutoPilot::StateMonitor::pos_cb, state);

    //Create object for arming service
    ros::ServiceClient arming_client = node.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    //Create object for set mode service
    ros::ServiceClient set_mode_client = node.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");  

    //Declare Setpoint Publisher
    ros::Publisher local_pos_pub = node.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = node.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);


    // wait for FCU connection
    while(ros::ok() && !state->state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // Generate blank message for publishing blank setpoints
    geometry_msgs::PoseStamped height;
    height.pose = AutoPilot::Pose(0,0,0);

    //send a few setpoints before starting as required for offboard mode
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(height);
        ros::spinOnce();
        rate.sleep();
    }

    // Set offboard mode
    while(state->state.mode != "OFFBOARD" && ros::ok()){
        mode.offboard_set_mode(set_mode_client);
        ros::spinOnce();
        rate.sleep();
    }

    // Arm the drone
    while(state->state.armed!=true){
        mode.setArm(arming_client);
        ros::spinOnce();
        rate.sleep();
    }

    height.pose = AutoPilot::Pose(0,0,10);
    geometry_msgs::TwistStamped vel;
    
    // Loop to send the setpoints to the vehicle via MAVROS
    while(ros::ok())
    {
        while(!((std::abs(state->pose.pose.position.x - height.pose.position.x) < 0.5) && 
                (std::abs(state->pose.pose.position.y - height.pose.position.y) < 0.5) && 
                (std::abs(state->pose.pose.position.z - height.pose.position.z) < 0.5))) //Keep publishing setpoints while comparing the current position with the setpoint within a given tolerance
        {
            local_pos_pub.publish(height);
            ros::spinOnce();
            rate.sleep();
        }
        for(int theta = 0; theta < 360; theta++)
        {
            vel.twist.linear.x = cos(theta*3.14/180);
            vel.twist.linear.y = sin(theta*3.14/180);
            vel.twist.angular.z = 0;
            local_vel_pub.publish(vel);
            ros::spinOnce();
            rate.sleep();
        }
        {
            local_vel_pub.publish(vel);
            ros::spinOnce();
            rate.sleep();
        }
        while(state->state.mode != "AUTO.LAND" && ros::ok()){
            mode.land_set_mode(set_mode_client);
            ros::spinOnce();
            rate.sleep();
        }
        while(state->state.armed!=false)
        {
            mode.setDisarm(arming_client);
            ros::spinOnce();
            rate.sleep();
        }
        break;
    }
    delete state;
    return 0;
}