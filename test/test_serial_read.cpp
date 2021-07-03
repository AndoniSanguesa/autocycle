#include <ros/ros.h>
#include <std_msgs/Float32.h>

void readState(std_msgs::Float32 msg){
    ROS_INFO_STREAM("State: " << msg.data);
}

void readRoll(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Roll: " << msg.data);
}

void readSteer(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Steer: " << msg.data);
}

void readDroll(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Droll: " << msg.data);
}

void readDsteer(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Dsteer: " << msg.data);
}

void readVel(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Vel: " << msg.data);
}

void readHead(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Heading: " << msg.data);
}

void readDhead(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Dheading: " << msg.data);
}

void readMet(std_msgs::Float32 msg){
    ROS_INFO_STREAM("Met: " << msg.data);
}

int main(int argc, char **argv){
    // Registers the node with the master
    ros::init(argc, argv, "test_serial_read");
    ros::NodeHandle nh;

    // Creates subscribers for all data types
    ros::Subscriber state_sub = nh.subscribe("sensors/state", 1, &readState);
    ros::Subscriber roll_sub = nh.subscribe("sensors/roll", 1, &readRoll);
    ros::Subscriber steer_sub = nh.subscribe("sensors/steer", 1, &readSteer);
    ros::Subscriber droll_sub = nh.subscribe("sensors/droll", 1, &readDroll);
    ros::Subscriber dsteer_sub = nh.subscribe("sensors/dsteer", 1, &readDsteer);
    ros::Subscriber vel_sub = nh.subscribe("sensors/vel", 1, &readVel);
    ros::Subscriber head_sub = nh.subscribe("sensors/heading", 1, &readHead);
    ros::Subscriber dhead_sub = nh.subscribe("sensors/dheading", 1, &readDhead);
    ros::Subscriber met_sub = nh.subscribe("sensors/met", 1, &readMet);

    ros::spin();
}
