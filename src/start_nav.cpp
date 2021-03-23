// This program handles the communication between different parts of
// Autocycle's navigation systems.
#include <ros/ros.h>
#include <autocycle_extras/LvxData.h>
#include <autocycle_extras/ObjectList.h>
#include <autocycle_extras/Object.h>
#include <autocycle_extras/GetData.h>
#include <autocycle_extras/RollAdj.h>
#include <autocycle_extras/DetectObjects.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

using namespace std::chrono;

bool ready = false;

void update_ready(const std_msgs::Empty msg){
    ready = true;
}

// Creates the subscriber that checks for when it is safe to continue
ros::Subscriber read_sub;

// TEMPRORARY UNTIL LVX ANALYSIS ALGORITHM IS COMPLETE.
// ONLY TO TEST THAT FRAMES ARE BEING RECORDED.
ros::ServiceClient lvx_client;

// Creates the service that will fetch the latest data
ros::ServiceClient get_data_client;

ros::ServiceClient detection_client;

// Creates service client that will call on the fix_roll service to fix the roll...
ros::ServiceClient roll_client;

// The response and request objects that will contain data regarding the lvx file
autocycle_extras::LvxData::Request lvx_req;
autocycle_extras::LvxData::Response lvx_resp;

// The response and request objects that will handle fetching data
autocycle_extras::GetData::Request get_data_req;
autocycle_extras::GetData::Response get_data_resp;

// The response and request objects that will handle object detection
autocycle_extras::DetectObjects::Request detect_req;
autocycle_extras::DetectObjects::Response detect_resp;

// The response and requests objects that will contain the non roll-adjusted points and the adjusted ones
autocycle_extras::RollAdj::Request adj_roll_req;
autocycle_extras::RollAdj::Response adj_roll_resp;

// Initializes the variable that will hold the lvx file
std::ofstream f_done;

// Initializes variables for time
double distance;
float velocity;
bool result;

std::string path_to_lvx = "f_done.lvx";

bool collect_data(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp
    ){
    ROS_INFO_STREAM("Sending request for LVX file.");

    // Waits until f_done.lvx has been populated
    f_done.open("f_done.lvx", std::ios::trunc);
    while(f_done.tellp() == 0){
      f_done.close();
      if(!ros::ok()){
        f_done.open("f_done.lvx", std::ios::trunc);
        f_done.write("done", 4);
        f_done.close();
        return 0;
      }
      f_done.open("f_done.lvx", std::ios::app);
    }
    f_done.close();

    // Collects the latest roll data
    get_data_req.data_type = "roll";
    result = get_data_client.call(get_data_req, get_data_resp);

    ROS_INFO_STREAM("LVX file generated.");
    ROS_INFO_STREAM("Sending request to analyze LVX File.");

    // Parses the lvx file
    lvx_req.path = path_to_lvx;
    result = lvx_client.call(lvx_req, lvx_resp);
    ROS_INFO_STREAM("NUMBER OF POINTS: " << lvx_resp.data.size());
    if(!result){
        exit(1);
    }

    ROS_INFO_STREAM("LVX file analyzed.");

    adj_roll_req.in = lvx_resp.data;
    adj_roll_req.roll = get_data_resp.data;
    result = roll_client.call(adj_roll_req, adj_roll_resp);

    ROS_INFO_STREAM("Points have been adjusted for roll.");

    ROS_INFO_STREAM("Sending LiDAR data to Object Detection");
    detect_req.data = adj_roll_resp.out;
    result = detection_client.call(detect_req, detect_resp);

    // Clears f_done.lvx file while waiting for the rest of the loop to be ready
    f_done.open(path_to_lvx, std::ios::trunc);
    f_done.close();
    return true;
}

int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Wait for the parse_lvx service to be active
  ros::service::waitForService("parse_lvx");

  // Waits for the data getter service to be active
  ros::service::waitForService("get_data");

  // Waits for the object detector service to be active
  ros::service::waitForService("object_detection");

  // Wait for the fix_roll service to be active
  ros::service::waitForService("fix_roll");

  // Creates the subscriber that checks for when it is safe to continue
  read_sub = nh.subscribe("cycle/ready", 1, &update_ready);

  // TEMPRORARY UNTIL LVX ANALYSIS ALGORITHM IS COMPLETE.
  // ONLY TO TEST THAT FRAMES ARE BEING RECORDED.
  lvx_client = nh.serviceClient<autocycle_extras::LvxData>("parse_lvx");

  // Creates the service that will fetch the latest data
  get_data_client = nh.serviceClient<autocycle_extras::GetData>("get_data");

  detection_client = nh.serviceClient<autocycle_extras::DetectObjects>("object_detection");

  // Creates service client that will call on the fix_roll service to fix the roll...
  roll_client = nh.serviceClient<autocycle_extras::RollAdj>("fix_roll");

  ros::ServiceServer data_server = nh.advertiseService("collect_data", &collect_data);

  // Navigation loop
  ros::spin();

  f_done.open(path_to_lvx, std::ios::trunc);
  f_done.write("done", 4);
  f_done.close();
}
