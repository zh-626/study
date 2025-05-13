#include "my_minimal_nodes/square.h"

offboard_class_timer::offboard_class_timer(ros::NodeHandle* nodehandle) 
    : nh(*nodehandle), rate(20.0)// 初始化rate为20Hz
{
    init_publisher();
    init_subscriber();
    init_service();
    
    // 创建定时器，每50ms调用一次calc_cb函数
    calc_timer = nh.createTimer(ros::Duration(0.05), &offboard_class_timer::calc_cb, this);
    
    // 初始化offboard模式和解锁命令
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    
    // 发送初始位置命令，确保PX4接收到足够的命令
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Offboard class initialized");
}

void offboard_class_timer::init_publisher()
{
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
}

void offboard_class_timer::init_subscriber()
{
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, 
                                                             &offboard_class_timer::local_pos_cb, this);
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, 
                                                  &offboard_class_timer::state_cb, this);
}

void offboard_class_timer::init_service()
{
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void offboard_class_timer::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = *msg;
}

void offboard_class_timer::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void offboard_class_timer::calc_cb(const ros::TimerEvent&)
{
    // 确保FCU已连接
    if (!current_state.connected) {
        ROS_WARN_THROTTLE(1, "FCU not connected");
        return;
    }
    
    // 切换到offboard模式
    if (current_state.mode != "OFFBOARD" && 
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("Offboard enabled");
        } else {
            ROS_WARN("Failed to set OFFBOARD mode");
        }
        last_request = ros::Time::now();
    }
    
    // 解锁无人机
    if (!current_state.armed && 
        current_state.mode == "OFFBOARD" && 
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
        } else {
            ROS_WARN("Failed to arm vehicle");
        }
        last_request = ros::Time::now();
    }
    
    // 执行方形轨迹
    if (current_state.mode == "OFFBOARD" && current_state.armed) {
        execute_square_trajectory();
    }
    
    // 始终发布位置指令，保持与PX4的通信
    local_pos_pub.publish(pose);
}

void offboard_class_timer::execute_square_trajectory()
{
    switch (step)
    {
    case 0: // 起飞到2米
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        
        if (local_pos.pose.position.z > 1.9 && local_pos.pose.position.z < 2.1) {
            sametimes++;
            if (sametimes > 100) {
                sametimes = 0;
                step = 1;
                ROS_INFO("Moving to waypoint 1: (2, 0, 2)");
            }
        } else {
            sametimes = 0;
        }
        break;
        
    case 1: // 移动到(2, 0, 2)
        pose.pose.position.x = 2;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        
        if (local_pos.pose.position.x > 1.9 && local_pos.pose.position.x < 2.1) {
            sametimes++;
            if (sametimes > 100) {
                sametimes = 0;
                step = 2;
                ROS_INFO("Moving to waypoint 2: (2, 2, 2)");
            }
        } else {
            sametimes = 0;
        }
        break;
        
    case 2: // 移动到(2, 2, 2)
        pose.pose.position.x = 2;
        pose.pose.position.y = 2;
        pose.pose.position.z = 2;
        
        if (local_pos.pose.position.y > 1.9 && local_pos.pose.position.y < 2.1) {
            sametimes++;
            if (sametimes > 100) {
                sametimes = 0;
                step = 3;
                ROS_INFO("Moving to waypoint 3: (0, 2, 2)");
            }
        } else {
            sametimes = 0;
        }
        break;
        
    case 3: // 移动到(0, 2, 2)
        pose.pose.position.x = 0;
        pose.pose.position.y = 2;
        pose.pose.position.z = 2;
        
        if (local_pos.pose.position.x > -0.1 && local_pos.pose.position.x < 0.1) {
            sametimes++;
            if (sametimes > 100) {
                sametimes = 0;
                step = 4;
                ROS_INFO("Moving to waypoint 4: (0, 0, 2)");
            }
        } else {
            sametimes = 0;
        }
        break;
        
    case 4: // 移动到(0, 0, 2)
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;
        
        if (local_pos.pose.position.y > -0.1 && local_pos.pose.position.y < 0.1) {
            sametimes++;
            if (sametimes > 100) {
                sametimes = 0;
                step = 5;
                ROS_INFO("Initiating landing");
            }
        } else {
            sametimes = 0;
        }
        break;
        
    case 5: // 降落
        offb_set_mode.request.custom_mode = "AUTO.LAND";
        if (current_state.mode != "AUTO.LAND" && 
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("AUTO.LAND enabled");
            } else {
                ROS_WARN("Failed to set AUTO.LAND mode");
            }
            last_request = ros::Time::now();
        }
        break;
        
    default:
        break;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting offboard control node...");
    
    offboard_class_timer offboard_node(&nh);
    
    // 使用spinOnce配合定时器
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}