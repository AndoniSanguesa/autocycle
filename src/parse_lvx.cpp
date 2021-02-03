#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <autocycle/LvxData.h>
#include <autocycle/LvxPoint.h>
#include <chrono>
#include <math.h>
#include <cstring>
#include <stdio.h>
using namespace std;
using namespace std::chrono;

// The publisher that will publish frame data
ros::Publisher lvx_pub;

bool parseLVX(
        autocycle::LvxData::Request &req,
        autocycle::LvxData::Response &resp
) {
    auto start = high_resolution_clock::now();
    streampos size;
    int data_type, x, y, z;
    char * buff;
    ifstream file (req.path, ios::in|ios::binary|ios::ate);
    if (file.is_open()) {
        // Initialization
        size = file.tellg();
        file.seekg(0, ios::beg);

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
                            file.ignore(4);

                        // z val
                        file.read(buff, 4);
                        z = *((uint32_t *) buff);

                        //xs.push_back(x);
                        //zs.push_back(z);

                        // Ignores tag and reflexivity
                        file.ignore(2);

                        autocycle::LvxPoint msg;
                        msg.x = x;
                        msg.z = z;
                        lvx_pub.publish(msg);
                        //ROS_INFO_STREAM("Coordinates: (" << x << ", " << z << ").\n");
                    }
                default:
                    // We can collect this data later if we want
                    file.ignore(24);
            }
        }
    }
    else {
        ROS_ERROR_STREAM("File could not be opened");
        return false;
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    ROS_INFO_STREAM("Time to process: " <<  duration.count() << "\n");
    return true;
}

int main(int argc, char **argv) {
    // Initialize the node and register it with the master.
    ros::init(argc, argv, "parse_lvx");
    ros::NodeHandle nh;

    // Register Service Server with the master.
    ros::ServiceServer lvx_server = nh.advertiseService("parse_lvx", &parseLVX);

    // Register Publish with the master.
    lvx_pub = nh.advertise<autocycle::LvxPoint>("lidar/data", 14000);

    // Constantly checks for new data to process
    ros::spin();
}
