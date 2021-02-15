#include <ros/ros.h>
#include <autocycle/LVXData.h>
#include <autocycle/Point.h>
#include <string>
#include <math>

using namespace std;

// Finds the range for x and y values of the points in a frame of data
void get_max_min(vector<autocycle::Point> points, int *x_range[2],int *y_range[2]){
    for(int i = 0; i < points.size(); i++){
        if(points[i].x < x_range[0]){
            x_range[0] = points[i].x
        } else if(points[i].x > x_range[1]){
            x_range[1] = points[i].x
        }
        if(points[i].y < y_range[0]){
            y_range[0] = points[i].y
        } else if(points[i].y > y_range[1]){
            y_range[1] = points[i].y
        }
    }
}

int main(int argc, char** argv){
    // Registers node with the master.
    ros::init(argc, argv, "test_visualize_lidar");
    ros::NodeHandle nh;

    // Waits for the data getter service to be active
    ros::service::waitForService("get_data");

    // Creates the service that will fetch the latest data
    ros::ServiceClient lvx_client = nh.serviceClient<autocycle::LvxData>("parse_lvx");

    // The response and request objects that will contain data regarding the lvx file
    autocycle::LvxData::Request lvx_req;
    autocycle::LvxData::Response lvx_resp;

    // Sets variables for visualization
    int height, width = 0;

    if(argc == 1){
        width = 500;
        height = 500
    }else if(argc != 3){
        ROS_FATAL_STREAM("test_visualize_lidar must be run with no arguments or 2 arguments for width and height respectively!")
    }else{
        width = stoi(argv[1]);
        height = stoi(argv[2]);
    }

    while(ros::ok()){
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

        // Parses the lvx file
        lvx_req.path = path_to_lvx;
        result = lvx_client.call(lvx_req, lvx_resp);

        // Finds the minimum and maximum x and y values for a given frame
        int x_range [2];
        int y_range [2];
        get_max_min(lvx_resp.data, x_range, y_range);

        //



    }
    return 0;
}