#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <serial/serial.h>
#include <autocycle_extras/GetDelta.h>
#include <fstream>
#include <sstream>
#include <string>

#include <iostream>

using namespace std;

bool dummy(autocycle_extras::GetDelta::Request &req,
	   autocycle_extras::GetDelta::Response &resp){
    return true;
}

int main(int argc, char **argv){
  // Registers node with the master.
  ros::init(argc, argv, "read_serial");
  ros::NodeHandle nh;
  
  ofstream myfile;
  myfile.open("stream_log.txt");

  // Registers the appropriate publishers
  ros::Publisher state_pub = nh.advertise<std_msgs::Float32>("sensors/state", 1);
  ros::Publisher roll_pub = nh.advertise<std_msgs::Float32>("sensors/roll", 1);
  ros::Publisher steer_pub = nh.advertise<std_msgs::Float32>("sensors/steer", 1);
  ros::Publisher droll_pub = nh.advertise<std_msgs::Float32>("sensors/droll", 1);
  ros::Publisher dsteer_pub = nh.advertise<std_msgs::Float32>("sensors/dsteer", 1);
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("sensors/vel", 1);
  ros::Publisher torque_pub = nh.advertise<std_msgs::Float32>("sensors/torque", 1);
  ros::Publisher head_pub = nh.advertise<std_msgs::Float32>("sensors/heading", 1);
  ros::Publisher dhead_pub = nh.advertise<std_msgs::Float32>("sensors/dheading", 1);
  ros::Publisher met_pub = nh.advertise<std_msgs::Float32>("sensors/met", 1);

  // Initializing vars
  string temp = "";
  char back_char;
  std_msgs::Float32 to_pub;
  int cur_publisher = 0;
  string names [10] = {"state", "roll", "steer", "droll", "dsteer", "vel", "torque", "heading", "dheading", "met"};
  ros::Publisher publishers [10] = {state_pub, roll_pub, steer_pub, droll_pub, dsteer_pub, vel_pub, torque_pub, head_pub, dhead_pub, met_pub};

  // Creates serial object to read from
  // TODO: Figure out what to do with the timeout
  serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

  bool ready = false;
  while(!ready){
    temp.append(my_serial.read());
    
    if((char) temp.back() == '\n'){
      temp.pop_back();
      ROS_INFO_STREAM("READ LINE : " << temp);
      if(temp.find("setup") != -1){
         ROS_INFO_STREAM("I THINK THE LINES ARE THE SAME!!!!");
      ready = true;
      }
      temp.clear();
    }
  }

  ros::ServiceServer dummy_serv = nh.advertiseService("due_ready", dummy);

  my_serial.write("t1,10000;");  
  while(ros::ok()){
    temp.append(my_serial.read());
    back_char = (char) temp.back();
    if(back_char == '\t' || back_char == '\n'){
      temp.pop_back();
      istringstream temp_as_stream(temp);
      temp_as_stream >> to_pub.data;
      temp.clear();
      if (cur_publisher%16 < 10){
        ROS_INFO_STREAM("COLLECTED DATA");
        publishers[cur_publisher%16].publish(to_pub);
	myfile << names[cur_publisher%16];
	myfile << to_string(to_pub.data);
	myfile << endl;
      }
      cur_publisher++;
    }
  }
  myfile.close();
}
