#ifndef SQUARE_H
#define SQUARE_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

class offboard_class_timer
{
private:
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub;
    ros::Subscriber local_pos_sub, state_sub;
    ros::ServiceClient arming_client, set_mode_client;
    ros::Timer calc_timer;

    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped local_pos, pose;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    ros::Time last_request;
    int step =0 ;
    int sametimes = 0;
    ros::Rate rate; // 声明频率控制对象

    void init_publisher();
    void init_subscriber();
    void init_service();
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void calc_cb(const ros::TimerEvent&);
    void execute_square_trajectory();

public:
    offboard_class_timer(ros::NodeHandle* nodehandle);
};

#endif