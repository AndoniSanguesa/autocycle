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

    // output file
    ofstream f_done;

    // Loops until ROS quits
    while(ros::ok()){
        // Opens the livox output file
        f_done.open("f_done.lvx", ios::trunc);

        //loops until a new frame has been written to the file
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

        // Closes the file and publishes message announcing that a new frame is ready
        f_done.close();
        frame_ready_pub.publish(e);

        // Waits until data has been processed
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
