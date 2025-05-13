#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

double target_x = 0.0;
double target_y = 0.0;
double target_z = 0.0;

//回传状态信息
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

//回传位置信息
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test1_node");
    ros::NodeHandle nh;
    nh.param<double>("target_x", target_x, 0.0);
    nh.param<double>("target_y", target_y, 0.0);
    nh.param<double>("target_z", target_z, 0.0);

    // 订阅器和发布器
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 5, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 5, pose_cb);
    ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 5);

    // 服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    ros::Rate rate(20.0);

    // 等待飞控连接
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 初始化目标高度
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = target_x;
    target_pose.pose.position.y = target_y;
    target_pose.pose.position.z = target_z;


    // 预发送设定点
    for(int i = 100; ros::ok() && i > 0; --i) {
        target_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    bool is_armed = false; // 解锁状态标记

    while(ros::ok()) {
        // 持续发布目标高度（关键！维持Offboard模式）
        target_pose.pose.position.x = target_x;
        target_pose.pose.position.y = target_y;
        target_pose.pose.position.z = target_z;
        target_pos_pub.publish(target_pose);

        // 检测到Offboard模式且未解锁时执行解锁
        if(current_state.mode == "OFFBOARD" && !current_state.armed) {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed and taking off!");
                is_armed = true;
            }
        }

        // 可选：检查是否到达目标高度
        if(is_armed &&
            fabs(current_pose.pose.position.x - target_x) < 0.1 &&
            fabs(current_pose.pose.position.y - target_y) < 0.1 &&
            fabs(current_pose.pose.position.z - target_z) < 0.1) {
            ROS_INFO("Reached target altitude!");
            
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}