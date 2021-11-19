#include <ros/ros.h>
#include <vector>
#include <tuple>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

ofstream output_file;                                  // File to output data to
bool ready = false;                                    // True if new LiDAR frame is ready for analysis
ofstream f_done;                                       // Output file that will contain LiDAR info
string path_to_lvx = "f_done.lvx";                     // Path to the data file
float height_of_lidar = 763;                           // Height of the LiDAR in millimeters

int height = 2400;                              // vertical height of detection window in millimeters
int width = 10000;                              // horizontal width of detection window in millimeters
int half_height = height/2;                     // Half of the above variable
int half_width = width/2;                       // Half of the above variable

// Parses the lvx file and sets the current points vector
void parse_lvx(){
    streampos size;
    int data_type, x, y, z;
    char * buff;
    long long next;
    vector<float> rotated_point;
    ifstream file (path_to_lvx, ios::in|ios::binary|ios::ate);
    if (file.is_open()) {
        // Initialization
        size = file.tellg();
        file.seekg(0, ios::beg);

        //ROS_INFO_STREAM("LVX FILE SIZE: " << size);

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
                        z = *((uint32_t *) buff);

                        // y val. We can record it if we determine we need it
                        file.read(buff, 4);
                        x = -*((uint32_t *) buff);
                        // z val
                        file.read(buff, 4);
                        y = *((uint32_t *) buff);

                        //z = z*0.9994 - y*0.0349
                        //y = z*0.0349 + y*0.9994

                        // Ignores tag and reflexivity
                        file.ignore(2);

                        if(z !=0 && x > -half_width && x < half_width && y > -height_of_lidar && y < half_height){
                            output_file << "(" << x << "," << y << "," << z << ") ";
                        }
			//o_file << "(" << p.x << ", " << p.y << ", " << p.z << ") ";
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
    //o_file << endl;
}

// Callback function for the `frame_ready` topic that sets the ready variable
void update_ready(const std_msgs::Empty msg){
    ready = true;
}

int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;


  // Initailizes the output file
  string time_string = to_string(chrono::duration_cast<chrono::seconds>(chrono::system_clock::now().time_since_epoch()).count());
  output_file.open("/home/ubuntu/" + time_string + ".txt");

  // Creates subscriber that waits for new lidar frame to be ready
  ros::Subscriber ready_sub = nh.subscribe("cycle/frame_ready", 1, &update_ready);

  while(ros::ok()){
    if(ready){
        parse_lvx();
        break;
    }
  }
  output_file.close();
}