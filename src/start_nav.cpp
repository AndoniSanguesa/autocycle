// This program handles the communication between different parts of
// Autocycle's navigation systems.
#include <ros/ros.h>
#include <autocycle/LvxData.h>
#include <autocycle/ObjectList.h>
#include <autocycle/Object.h>
#include <autocycle/Roll.h>
#include <autocycle/Heading.h>
#include <autocycle/RollAdj.h>
#include <fstream>
#include <iostream>
#include <std_msgs/String.h>


std::string path_to_lvx = "f_done.lvx";


int main(int argc, char **argv) {
  // Will contain the status of service calls
  bool result;

  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Wait for the parse_lvx service to be active
  ros::service::waitForService("parse_lvx");

  // Waits for the roll getter service to be active
  ros::service::waitForService("get_roll");

  // Waits for the heading getter service to be active
  ros::service::waitForService("get_heading");

  // Wait for the fix_roll service to be active
  ros::service::waitForService("fix_roll");

  // Wait for the plan_path service to be active
  ros::service::waitForService("plan_path");

  // TEMPRORARY UNTIL LVX ANALYSIS ALGORITHM IS COMPLETE.
  // ONLY TO TEST THAT FRAMES ARE BEING RECORDED.
  ros::ServiceClient lvx_client = nh.serviceClient<autocycle::LvxData>("parse_lvx");

  // Creates the service that will fetch the latest roll data
  ros::ServiceClient get_roll_client = nh.serviceClient<autocycle::Roll>("get_roll");

  // Creates the service client that will fetch the latest heading data
  ros::ServiceClient get_heading_client = nh.serviceClient<autocycle::Heading>("get_heading");

  // Creates service client that will call on the fix_roll service to fix the roll...
  ros::ServiceClient roll_client = nh.serviceClient<autocycle::RollAdj>("fix_roll");

  //TEMPORARY UNTIL I N T E G R A T I O N
  ros::ServiceClient path_client = nh.serviceClient<autocycle::ObjectList>("plan_path");

  // The response and request objects that will contain data regarding the lvx file
  autocycle::LvxData::Request lvx_req;
  autocycle::LvxData::Response lvx_resp;

  // The response and request objects that will handle fetching roll data
  autocycle::Roll::Request get_roll_req;
  autocycle::Roll::Response get_roll_resp;

  // The response and reqeust objects that will handle fetching heading data
  autocycle::Heading::Request get_heading_req;
  autocycle::Heading::Response get_heading_resp;

  // The response and requests objects that will contain the non roll-adjusted points and the adjusted ones
  autocycle::RollAdj::Request adj_roll_req;
  autocycle::RollAdj::Response adj_roll_resp;

  // The response and request objects that will contain data regarding the path
  autocycle::ObjectList::Request path_req;
  autocycle::ObjectList::Response path_resp;

  // Initializes the variable that will hold the lvx file
  std::ofstream f_done;

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

    // Collects the latest roll data
    result = get_roll_client.call(get_roll_req, get_roll_resp);

    ROS_INFO_STREAM("LVX file generated.");
    ROS_INFO_STREAM("Sending request to analyze LVX File.");

    // Publishes the determined lvx file to LiDAR/path
    lvx_req.path = path_to_lvx;
    result = lvx_client.call(lvx_req, lvx_resp);
    ROS_INFO_STREAM("NUMBER OF POINTS: " << lvx_resp.data.size());
    if(!result){
        exit(1);
    }

    ROS_INFO_STREAM("LVX file analyzed.");

    adj_roll_req.in = lvx_resp.data;
    adj_roll_req.roll = get_roll_resp.roll;
    result = roll_client.call(adj_roll_req, adj_roll_resp);

    ROS_INFO_STREAM("Points have been adjusted for roll.");

    ROS_INFO_STREAM("Sending Object data to path planning");
    ROS_WARN_STREAM("THIS IS TEMPORARY UNTIL WE CAN GET OBJECT DATA");

    // TEMPORARY: Sends dummy object data to bezier/objects to test for now
    autocycle::Object o1 ;
    autocycle::ObjectList ol1;
    o1.edge_to_path = 2;
    o1.dist_to_edge = 3;
    o1.edge_len = 3;
    path_req.obj_lst.push_back(o1);

    // Report parameterized equation for the calculated path
    path_client.call(path_req, path_resp);
    ROS_WARN_STREAM("THIS INFORMATION SHOULD ONLY BE REPORTED HERE UNTIL I N T E G R A T I O N");
    ROS_INFO_STREAM("Param for path: " << path_resp.param);

    // Clears f_done.lvx file
    f_done.open(path_to_lvx, std::ios::trunc);
    f_done.close();
  }
  f_done.open(path_to_lvx, std::ios::trunc);
  f_done.write("done", 4);
  f_done.close();
}
