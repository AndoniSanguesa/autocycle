#include <ros/ros.h>
#include <autocycle/LVXData.h>
#include <autocycle/Point.h>
#include <string>
#include <math>
#include <stdlib.h>

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

char get_char(float z_val){
    if(z_val < 1){
        return ' ';
    } else if (z_val < 1.3){
        return '░';
    } else if (z_val < 1.6){
        return '▒';
    } else if (z_val < 2){
        return '▓';
    } else{
        return '█';
    }
}

int main(int argc, char** argv){
    // Registers node with the master.
    ros::init(argc, argv, "test_visualize_lidar");
    ros::NodeHandle nh;

    // Waits for the data getter service to be active
    ros::service::waitForService("parse_lvx");

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

    // Initializes Vars
    float x_range [2];
    float y_range [2];
    float x_bin_size, y_bin_size;
    int x_bin_ind, y_bin_ind, count;
    float bin_count[width][height];
    float bins [width][height];
    vector<autocycle::Point> points;

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

        // Gets points from response object
        points = lvx_resp.data

        // Finds the minimum and maximum x and y values for a given frame
        get_max_min(points, x_range, y_range);

        // Calculates bin size
        x_bin_size = (x_range[1]-x_range[0])/width;
        y_bin_size = (y_range[1]-y_range[0])/height;

        // Calculates running z averages for bins
        for(int i = 0; i < points.size(); i++){
            // Calculates the appropriate bin for the given x and y value
            x_bin_ind = floor((points[i].x - x_range[0])/x_bin_size);
            y_bin_ind = floor((points[i].y - y_range[0])/y_bin_size);

            // Gets the current number of points used in the bin
            count = bin_count[x_bin_ind][y_bin_ind];

            // Increases the count of points in the bin
            bin_count[x_bin_ind][y_bin_ind]++;

            // Updates the z value in the final bin matrix
            bins[x_bin_ind][y_bin_ind] = (bins[x_bin_ind][y_bin_ind] * count + points[i].y) / (count+1);
        }

        // Generates image
        system ("CLS");
        for(int i = 0; i < height; i++){
            for(int j = 0; j < width; j++){
                cout << get_char(bins[j][i]);
            }
            cout << endl;
        }
    }
    return 0;
}