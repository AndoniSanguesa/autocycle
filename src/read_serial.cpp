#include <ros/ros.h>
#include <autocycle_extras/Data.h>
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

  // Registers the appropriate publishers
  ros::Publisher data_pub = nh.advertise<autocycle_extras::Data>("sensors/data", 1);

  // Initializing vars
  string temp = "";
  char back_char;
  autocycle_extras::Data to_pub;
  int cur_ind = 0;
  // string names [10] = {"state", "roll", "steer", "droll", "dsteer", "vel", "torque", "heading", "dheading", "met"};

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

  while(ros::ok()){
    temp.append(my_serial.read());
    back_char = (char) temp.back();
    if(back_char == '\t' || back_char == '\n'){
      temp.pop_back();
      to_pub.push_back(stof(temp));
      temp.clear();
      if (cur_ind%16 == 15){
        data_pub.publish(to_pub);
        to_pub.clear();
      }
      cur_ind++;
    }
  }
}
