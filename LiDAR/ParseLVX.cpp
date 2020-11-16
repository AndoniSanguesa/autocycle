#include <iostream>
#include <fstream>
#include <chrono>
using namespace std;
using namespace std::chrono;

int main() {
    auto start = high_resolution_clock::now();
    streampos size;
    uint64_t next_offset;
    int frame_cnt;
    int data_type, x, y, z;
    char * buff;

    ifstream file ("lidar.bin", ios::in|ios::binary|ios::ate);
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
        frame_cnt = 0;
        while(file.tellg() < size) {
            // Ignoring current frame's absolute offset
            file.ignore(8);

            // Offset for the next frame
            file.read(buff, 8);
            next_offset = *((uint64_t*) buff);

            // Ignoring current frame's index
            file.ignore(8);

            // Analyzes Packages
            while(file.tellg() < next_offset){
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
                if(data_type == 2){
                    for(int i = 0; i < 96; i++){
                        frame_cnt++;
                        // x val
                        file.read(buff, 4);
                        x = *((uint32_t *)buff);

                        // y val
                        file.read(buff, 4);
                        y = *((uint32_t *)buff);

                        // z val
                        file.read(buff, 4);
                        z = *((uint32_t *)buff);

                        //TODO: Define a data structure for x, y, & z
                        //TODO: Decide if we want any more data (reflexivity, tag, or IMU)


                        // Ignores tag and reflexivity
                        file.ignore(2);
                    }
                } else if(data_type == 6){
                    // We can collect this data later if we want
                    file.ignore(24);
                }
            }
        }
    }
    else cout << "File could not be opened";
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << duration.count() << endl;
    return 0;
}