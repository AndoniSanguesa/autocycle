#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <chrono>

int main(int argc, char** argv){
    // Registers node with the master
    ros::init(argc, argv, "test_sim");
    ros::NodeHandle nh;

    // Creates publishers to simulate sensor data
    ros::Publisher roll_pub = nh.advertise<std_msgs::Float32>("sensors/roll", 1);
    ros::Publisher met_pub = nh.advertise<std_msgs::Float32>("sensors/met", 1);
    ros::Publisher heading_pub = nh.advertise<std_msgs::Float32>("sensors/heading", 1);
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("sensors/vel", 1);

    std_msgs::Float32 to_pub;
    typedef std::chrono::high_resolution_clock clock;
    auto t0 = clock::now();
    auto t1 = clock::now();

    // Simulates sensor data
    while(ros::ok()){
        t1 = clock::now();
        to_pub.data = (std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)).count();
        met_pub.publish(to_pub);
	std_msgs::Float32 to_pub;
        to_pub.data = 0;
        roll_pub.publish(to_pub);
        heading_pub.publish(to_pub);
        vel_pub.publish(to_pub);
    }
}
