#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <fstream>
#include <iostream>

using namespace std;

int main(int argc, char **argv){
    // Registers node with the master
    ros::init(argc, argv, "frame_ready");
    ros::NodeHandle nh;

    // Creates subscriber
    ros::Publisher frame_ready_pub = nh.advertise<std_msgs::Empty>("cycle/frame_ready", 1);

    // Creates message to publish
    std_msgs::Empty e;

    ofstream f_done;

    while(ros::ok()){
        f_done.open("f_done.lvx", ios::trunc);
        while(f_done.tellp() == 0){
          f_done.close();
          if(!ros::ok()){
            f_done.open("f_done.lvx", ios::trunc);
            f_done.write("done", 4);
            f_done.close();
            return 0;
          }
          f_done.open("f_done.lvx", ios::app);
        }
        f_done.close();
        frame_ready_pub.publish(e);
        while(f_done.tellp() != 0){
          f_done.close();
          if(!ros::ok()){
            f_done.open("f_done.lvx", ios::trunc);
            f_done.write("done", 4);
            f_done.close();
            return 0;
          }
          f_done.open("f_done.lvx", ios::app);
        }
        f_done.close();
    }
}
