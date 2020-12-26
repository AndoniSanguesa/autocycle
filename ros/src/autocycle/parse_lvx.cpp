#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <autocycle/LvxData.h>
#include <chrono>
#include <math.h>
#include <cstring>
using namespace std;
using namespace std::chrono;

// These Vectors will contain the data points
vector<int> xs;
vector<int> zs;

// The publisher that will publish frame data
ros::Publisher lvx_pub;

unsigned int get_pow_2(int power){
    return((unsigned int) pow(2, power));
}

void check_status(uint32_t status_code){
    cout << "---------------------------------------------\n";
    cout << "              STATUS REPORT                  \n";
    cout << "---------------------------------------------\n";

    // Check Temperature
    if(get_pow_2(0) & status_code){
        cout << "Temperature : HIGH\n";
    } else if (get_pow_2(1) & status_code){
        cout << "Temperature : EXTREMELY HIGH\n";
    } else{
        cout << "Temperature : GOOD\n";
    }

    // Check Voltage
    if(get_pow_2(2) & status_code){
        cout << "Voltage : HIGH\n";
    } else if (get_pow_2(3) & status_code){
        cout << "Voltage : EXTREMELY HIGH\n";
    } else{
        cout << "Voltage : GOOD\n";
    }

    // Motor Status
    if(get_pow_2(4) & status_code){
        cout << "Motor : WARNING\n";
    } else if (get_pow_2(5) & status_code){
        cout << "Motor : CRITICAL WARNING, UNABLE TO FUNCTION\n";
    } else {
        cout << "Motor : GOOD\n";
    }

    // Dirty Warning
    if(get_pow_2(6) & status_code){
        cout << "Sensor : DIRTY\n";
    } else {
        cout << "Sensor : CLEAN\n";
    }

    // BIT 7 IS RESERVED AND CONTAINS NO DATA

    // Firmware Warning
    if(get_pow_2(8) & status_code){
        cout << "Firmware : PLEASE UPDATE\n";
    } else {
        cout << "Firmware : OK\n";
    }

    // PPS Status (For  GPS Synchronization)
    if(get_pow_2(9) & status_code){
        cout << "PPS : OK\n";
    } else{
        cout << "PPS : NO PPS SIGNAL\n";
    }

    // Device Status
    if(get_pow_2(10) & status_code){
        cout << "Device : APPROACHING END OF LIFE SERVICE\n";
    } else{
        cout << "Device : GOOD\n";
    }

    // Fan Status
    if(get_pow_2(11) & status_code){
        cout << "Fan : WARNING\n";
    } else{
        cout << "Fan : GOOD\n";
    }

    // Self Heating. LiDAR will heat itself under cold temperatures
    if(get_pow_2(12) & status_code){
        cout << "Self Heating : OFF\n";
    } else{
        cout << "Self Heating : ON\n";
    }

    // PTP Status (For GPS Synchronization)
    if(get_pow_2(13) & status_code){
        cout << "PTP : 1588 SIGNAL OK\n";
    } else{
        cout << "PTP : NO 1588 SIGNAL\n";
    }

    // Time Synchronization
    if(get_pow_2(16) & status_code){
        cout << "Time Sync : ABNORMAL\n";
    } else if((get_pow_2(14) & status_code) && (get_pow_2(15) & status_code)){
        cout << "Time Sync : USING PPS SYNCHRONIZATION\n";
    } else if(get_pow_2(15) & status_code){
        cout << "Time Sync : USING GPS SYNCHRONIZATION\n";
    } else if(get_pow_2(14) & status_code){
        cout << "Time Sync : USING PTP 1588 SYNCHRONIZATION\n";
    } else{
        cout << "Time Sync : SYSTEM DOES NOT START TIME SYNCHRONIZATION\n";
    }

    // BYTES 17-29 INCLUSIVE ARE RESERVED AND CONTAIN NO INFORMATION

    // System Status Summary
    if(get_pow_2(30) & status_code){
        cout << "System Summary : WARNING\n";
    } else if (get_pow_2(31) & status_code){
        cout << "System Summary : ERROR\n";
    }else{
        cout << "System Summary : NORMAL\n";
    }
    cout << "---------------------------------------------\n";
}

void parseLVX(const std_msgs::String &msg) {
    auto start = high_resolution_clock::now();
    streampos size;
    uint64_t next_offset;
    int frame_cnt;
    int data_type, x, y, z;
    char * buff;

    ifstream file (msg.data, ios::in|ios::binary|ios::ate);
    if (file.is_open()) {

        // Initialization
        size = file.tellg();
        file.seekg(0, ios::beg);

        // Validation
        buff = new char[10];
        file.read(buff, 10);
        if(strcmp(buff, "livox_tech") != 0) {
            throw;
        }

        // Garbage validation stuff we don't need and some variables that I'm ignoring
        // VARIABLES BEING SKIPPED
        // Frame Duration
        // Device Count
        // Broadcast Code
        // Hub SN Code
        // Device Index/Type
        // External Toggle
        // IMU data (should all init to 0)
        file.ignore(78);

        // Analyzes Frames
        //frame_cnt = 0;
        while(file.tellg() < size && file.tellg() != -1) {
            // Ignoring current frame's absolute offset
            file.ignore(8);

            // Offset for the next frame
            file.read(buff, 8);
            next_offset = *((uint64_t *) buff);

            // Ignoring current frame's index
            file.ignore(8);

            // Analyzes Packages
            while (file.tellg() < next_offset) {
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

                            xs.push_back(x);
                            zs.push_back(z);

                            // Ignores tag and reflexivity
                            file.ignore(2);
                        }
                        break;
                    default:
                        // We can collect this data later if we want
                        file.ignore(24);
                        break;
                }
            }
            autocycle::LvxData msg;
            msg.xs = xs;
            msg.zs = zs;
            lvx_pub.publish(msg);
            xs.clear();
            zs.clear();
        }
    }
    else cout << "File could not be opened";
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << duration.count() << endl;
}

int main(int argc, char **argv) {
    // Initialize the node and register it with the master.
    ros::init(argc, argv, "parse_lvx");
    ros::NodeHandle nh;

    // Register subscriber looking for lvx files
    ros::Subscriber lvx_sub = nh.subscribe("LiDAR/path", 1, &parseLVX);

    // Register a publisher with the master.
    lvx_pub = nh.advertise<autocycle::LvxData>("LiDAR/data", 3);

    // Constantly checks for new data to process
    ros::spin();
}
