#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <demo_01/cir.h>

mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;
tf::Quaternion jiao;


double roll,pitch,yaw;

bool is_at_position(double x, double y, double z, double offset=0.5)
{
    double dx = local_pos.pose.pose.position.x - x;
    double dy = local_pos.pose.pose.position.y - y;
    double dz = local_pos.pose.pose.position.z - z;
    return sqrt(dx*dx + dy*dy + dz*dz) < offset;
}





void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation,jiao);
    tf::Matrix3x3(jiao).getRPY(roll,pitch,yaw);
    ROS_INFO("X:%f",local_pos.pose.pose.position.x);
    ROS_INFO("Y:%f",local_pos.pose.pose.position.y);
    ROS_INFO("Z:%f",local_pos.pose.pose.position.z);
    ROS_INFO("roll:%f",roll);
    ROS_INFO("pitch:%f",pitch);
    ROS_INFO("yaw:%f",yaw);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom",10,local_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    CircularTrajectory trajectory(5.0, 0.0 , 0.0 , 10 , 0.2);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 10;

 

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();


   //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        double elapsed = (ros::Time::now() - start_time).toSec();
        pose = trajectory.calculate_pose(elapsed);

        // 新增：设置yaw朝向运动方向
        double yaw = atan2(pose.pose.position.y - local_pos.pose.pose.position.y,
                          pose.pose.position.x - local_pos.pose.pose.position.x);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
