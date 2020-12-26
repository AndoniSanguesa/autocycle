// This program acts as an intermediary node to call on the LiDAR to record frames.
#include <ros/ros.h>
#include <autocycle/Lidar.h>
#include <experimental/filesystem>
#include <unistd.h>
#include <string>
namespace fs = std::experimental::filesystem;
using namespace std;

// Value of __FILE__ must be string. This contains the full path to current file
// including the file name. I am not using an absolute path so that I can make
// this more reproducible.
string file (__FILE__);

// Path to current file.
string path_to_file = file.substr(0, file.length() - 13);

// Path to where the LVX file will be generated.
string path_to_lvx = path_to_file + "./main/LiDAR/Livox-SDK/build/sample/lidar_lvx_file/";

// Callback function whenever the run_lidar service is called.
// TODO: This function could hang if livox fails to record.
//       Figure out how to deal with this!
bool callLidar(
  autocycle::Lidar::Request &req,
  autocycle::Lidar::Response &resp
) {
  // Runs the command to run the LiDAR.
  system((path_to_lvx + "lidar_lvx_sample -t " + to_string(req.time)).c_str());
  sleep(req.time);

  // Whether the LVX file has been generated or not
  bool found = false;

  //Checks for newly generated LiDAR file
  while(!found){
    for (const auto & entry : fs::directory_iterator(path_to_lvx)){
      if(isdigit(((string) entry.path()).at(0))){
        resp.path = path_to_lvx + ((string) entry.path());
        found = true;
        break;
      }
    }
  }
  return true;
}

int main(int argc, char **argv) {
  // Initialize the node and register it with the master.
  ros::init(argc, argv, "run_lidar");
  ros::NodeHandle nh;

  // Register service with the master.
  ros::ServiceServer lidar_server = nh.advertiseService(
    "run_lidar", &callLidar);

  // Checks for messages to read
  ros::spin();
}
