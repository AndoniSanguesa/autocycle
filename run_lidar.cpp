// This program acts as an intermediary node to call on the LiDAR to record frames.
#include <ros/ros.h>
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
string path_to_lvx = path_to_file + "/Livox-SDK/build/sample/lidar_lvx_file/";


int main(int argc, char **argv) {
  // Initialize the node and register it with the master.
  ros::init(argc, argv, "run_lidar");
  ros::NodeHandle nh;

  // Runs LiDAR
  system((path_to_lvx + "lidar_lvx_sample").c_str());
}
