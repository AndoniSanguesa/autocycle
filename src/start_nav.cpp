// This program handles the communication between different parts of
// Autocycle's navigation systems.
#include <ros/ros.h>
#include <autocycle/LvxData.h>
#include <autocycle/ObjectList.h>
#include <autocycle/Object.h>
#include <autocycle/GetData.h>
#include <autocycle/RollAdj.h>
#include <autocycle/DetectObjects.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <std_msgs/String.h>

using namespace std::chrono;

std::string path_to_lvx = "f_done.lvx";


int main(int argc, char **argv) {
  // Will contain the status of service calls
  bool result;

  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Wait for the parse_lvx service to be active
  ros::service::waitForService("parse_lvx");

  // Waits for the data getter service to be active
  ros::service::waitForService("get_data");

  // Waits for the object detector service to be active
  ros::service::waitForService("object_detection")

  // Wait for the fix_roll service to be active
  ros::service::waitForService("fix_roll");


  // TEMPRORARY UNTIL LVX ANALYSIS ALGORITHM IS COMPLETE.
  // ONLY TO TEST THAT FRAMES ARE BEING RECORDED.
  ros::ServiceClient lvx_client = nh.serviceClient<autocycle::LvxData>("parse_lvx");

  // Creates the service that will fetch the latest data
  ros::ServiceClient get_data_client = nh.serviceClient<autocycle::GetData>("get_data");

  ros::ServiceClient detection_client = nh.serviceClient<autocycle::DetectObjects>("object_detection")

  // Creates service client that will call on the fix_roll service to fix the roll...
  ros::ServiceClient roll_client = nh.serviceClient<autocycle::RollAdj>("fix_roll");

  // The response and request objects that will contain data regarding the lvx file
  autocycle::LvxData::Request lvx_req;
  autocycle::LvxData::Response lvx_resp;

  // The response and request objects that will handle fetching data
  autocycle::GetData::Request get_data_req;
  autocycle::GetData::Response get_data_resp;

  // The response and request objects that will handle object detection
  autocycle::DetectObjects::Request detect_req;
  autocycle::DetectObjects::Response detect_resp;

  // The response and requests objects that will contain the non roll-adjusted points and the adjusted ones
  autocycle::RollAdj::Request adj_roll_req;
  autocycle::RollAdj::Response adj_roll_resp;

  // Initializes the variable that will hold the lvx file
  std::ofstream f_done;

  // Initializes variables for time
  high_resolution_clock::time_point start;
  high_resolution_clock::time_point end;
  double duration, distance;
  float velocity;

  // Navigation loop
  while(ros::ok()){
    ROS_INFO_STREAM("Sending request for LVX file.");

    // Waits until f_done.lvx has been populated
    f_done.open("f_done.lvx", std::ios::app);
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
    start = high_resolution_clock::now();

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

    ROS_INFO_STREAM("Sending LiDAR data to Object Detection")
    detect_req.data = adj_roll_resp.out
    result = detection_client.call(detect_req, detect_resp)

    // Clears f_done.lvx file
    f_done.open(path_to_lvx, std::ios::trunc);
    f_done.close();
  }
  f_done.open(path_to_lvx, std::ios::trunc);
  f_done.write("done", 4);
  f_done.close();
}
