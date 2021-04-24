#include <ros/ros.h>
#include <autocycle_extras/LvxData.h>
#include <autocycle_extras/ObjectList.h>
#include <autocycle_extras/Object.h>
#include <autocycle_extras/Point.h>
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
#include <std_msgs/Float32.h>
#include <stdio.h>
using namespace std::chrono;

bool ready = false;

void update_ready(const std_msgs::Empty msg){
    ready = true;
}

// Creates the subscriber that checks for when it is safe to continue
ros::Subscriber read_sub;

ros::ServiceClient detection_client;

// Creates service client that will call on the fix_roll service to fix the roll...
ros::ServiceClient roll_client;

std::vector<autocycle_extras::Point> points;

// The response and request objects that will handle object detection
autocycle_extras::DetectObjects::Request detect_req;
autocycle_extras::DetectObjects::Response detect_resp;

// Initializes the variable that will hold the lvx file
std::ofstream f_done;

// Initializes variables for time
float roll = 0;
bool result;

std::string path_to_lvx = "f_done.lvx";

void update_roll(const std_msgs::Float32 data){
    roll = data.data;
}

void parse_lvx(){
    std::streampos size;
    int data_type, x, y, z;
    char * buff;
    long long next;

    std::ifstream file (path_to_lvx, ios::in|ios::binary|ios::ate);
    if (file.is_open()) {
        // Initialization
        size = file.tellg();
        file.seekg(0, std::ios::beg);

        ROS_INFO_STREAM("LVX FILE SIZE: " << size);

        buff = new char[4];

        while (file.tellg() != -1) {
  	    // Ignore the following data
            // Device Index for this frame
            // Package Protocol Version
            // Slot ID (Used only for Livox Mid)
            // LiDAR ID
            // Status Code (This is important for assessing physical issues with the LiDAR. For now use the python version to check the status)
            // Time Stamp Type
            file.ignore(10);

            // Data Type
            // 0 : Cartesian Coordinate System; Single Return; (Only for Livox Mid)
            // 1 : Spherical Coordinate System; Single Return; (Only for Livox Mid)
            // 2 : Cartesian Coordinate System; Single Return;
            // 3 : Spherical Coordinate System; Single Return;
            // 4 : Cartesian Coordinate System; Double Return;
            // 5 : Spherical Coordinate System; Double Return;
            // 6 : IMU Information
	    file.read(buff, 1);
            data_type = int((unsigned char) buff[0]);

	    //ROS_INFO_STREAM(data_type);
            // Ignoring Time Stamp
            file.ignore(8);
            // Analyze Data (Coordinate point or IMU)
            // Cartesian Coordinate System; Single Return
            switch (data_type) {
                case 2:
                    for (int i = 0; i < 96; i++) {
                        //frame_cnt++;
                        // x val
                        file.read(buff, 4);
                        x = *((uint32_t *) buff);

                        // y val. We can record it if we determine we need it
                        file.read(buff, 4);
                        y = *((uint32_t *) buff);

                        // z val
                        file.read(buff, 4);
                        z = *((uint32_t *) buff);

                        //xs.push_back(x);
                        //zs.push_back(z);

                        // Ignores tag and reflexivity
                        file.ignore(2);

                        autocycle_extras::Point p;
                        p.z = x;
                        p.x = y;
                        p.y = z;
			if(!(x == 0 && y == 0 && z ==0)){
			    points.push_back(p);
			}
                       // if(-50 < p.x and p.x < 50){
		       //     ROS_INFO_STREAM(p.z);
		       //}
		    }
		    break;
                default:
                    // We can collect this data later if we want
                    file.ignore(24);
            }
	    }
    }
    else {
        ROS_ERROR_STREAM("File could not be opened");
    }
    points.shrink_to_fit();
}

void fix_roll(){
  // Initializes variables
  autocycle_extras::Point p;
  autocycle_extras::Point np;

  // Updates each point in `req` and pushes it to the new vector
  for(i=0;i<points.size();i++){
    p = points[p];
    np.x = (p.x*cos(roll)) - (p.y*sin(roll));
    np.y = (p.x*sin(roll)) + (p.y*cos(roll));
    np.z = p.z;
    points[p] = np;
  }
}

bool collect_data(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp
    ){
    // ROS_INFO_STREAM("Sending request for LVX file.");
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

    //ROS_INFO_STREAM("LVX file generated.");
    //ROS_INFO_STREAM("Sending request to analyze LVX File.");

    // Parses the lvx file
    parse_lvx();
    //ROS_INFO_STREAM("LVX file analyzed.");

    auto start = std::chrono::high_resolution_clock::now();
    fix_roll();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    ROS_INFO_STREAM("NAV LOOP TOOK : " << (float) duration.count() / 1000.0 << " SECONDS");
    //ROS_INFO_STREAM("Points have been adjusted for roll.");

    //ROS_INFO_STREAM("Sending LiDAR data to Object Detection");
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

  // Waits for the object detector service to be active
  ros::service::waitForService("object_detection");

  // Creates the subscriber that checks for when it is safe to continue
  read_sub = nh.subscribe("cycle/ready", 1, &update_ready);

  // Creates subscriber for updating roll
  ros::Subscriber get_roll = nh.subscribe("sensors/roll", 1, &update_roll);

  detection_client = nh.serviceClient<autocycle_extras::DetectObjects>("object_detection");

  ros::ServiceServer data_server = nh.advertiseService("collect_data", &collect_data);

  points.reserve(15000);

  // Navigation loop
  ros::spin();

  f_done.open(path_to_lvx, std::ios::trunc);
  f_done.write("done", 4);
  f_done.close();
}
