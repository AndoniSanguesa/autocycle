#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <autocycle_extras/CalcDeltas.h>
#include <autocycle_extras/GPS.h>
#include <autocycle_extras/DesiredGPS.h>
#include <autocycle_extras/Data.h>
#include <cmath>
#include <set>
#include <vector>
#include <functional>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <serial/serial.h>
#include <sstream>
#include <string>
#include<bits/stdc++.h>
#include <queue>

using namespace std;

// Creates serial object to write to

serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

// General use variable initialization
tuple<float, float> bike_pos = {0, 0};
bool is_new_data = false;                              // True if there is new data to update object positions with
ofstream output_file;                                  // File to output data to
ofstream tab_file;                                     // File to output data to. but only telemetry and tab separated to satiate Misha's ungodly desire for tab separated data
bool ready = false;                                    // True if new LiDAR frame is ready for analysis
vector<tuple<float, float, float>> lvx_points;         // Contains points parsed from LiDAR data
ofstream f_done;                                       // Output file that will contain LiDAR info
string path_to_lvx = "f_done.lvx";                     // Path to the data file
float height_of_lidar = 763;                           // Height of the LiDAR in millimeters
vector<tuple<float, float, float, float>> obj_lst;     // Current list of objects
vector<tuple<float, float, float, float>> new_obj_lst; // Temporary vector used in adjusting state
vector<tuple<float, float, float, float>> cond_objs;   // Contains objects post-convex hull
float sync_head_amt = 0;                               // Amount to increase our heading by to synchronize with GPS
tuple<float, float> desired_gps_pos;                   // Position describing the next desired position to get to
bool desired_gps_set = false;                          // Whether or not a desired GPS position has been set

// ROS varibale initialization
ros::Publisher calc_deltas;                   // ROS publisher that publishes new paths for delta calculation
autocycle_extras::CalcDeltas calc_deltas_pub; // ROS message object that will be published by `calc_deltas`
autocycle_extras::DesiredGPS desired_gps_obj; // ROS object that contains the data provided to and from the corresponding client
ros::ServiceClient desired_gps_cli;           // ROS Service Client that will request next desired (long, lat)

// Sensor data variables
vector<float> data;                           // Contains latest data
tuple<float, float> cur_gps;                  // Latest longitude and latitude
tuple<float, float> prev_gps;                 // Previous longitude and latitude
tuple<float, float> r_hat = {0, 0};           // vector from back wheel ground patch to the lidar

// Object Detection Parameters

int height = 2400;                              // vertical height of detection window in millimeters
int width = 10000;                              // horizontal width of detection window in millimeters
float max_dist = 20000;                         // Maximum distance to detect objects for
int cell_dim = 50;                              // dimension of cells in millimeters (cells are squares)
int half_height = height/2;                     // Half of the above variable
int half_width = width/2;                       // Half of the above variable
int cell_row = ceil((1.0 * height) / cell_dim); // Number of cells in a row
int cell_col = ceil((1.0 * width) / cell_dim);  // Number of cells in a column
int col_diff = 50;                              // If the z value for 2 adjacent cells in a column differ by more than this variable, they are considered different objects
int for_jump_diff = col_diff * 1.5;             // Minimum negative change in z value to indicate an object
int counter_reps = 2;                           // Number of consecutive consistent column values needed to immediately determine that an object is present
int same_obj_diff = 150;                        // Horizontally adjacent Z distances under this value are considered the same object
int group_dist = 1500;                          // max dist between adjacent objects for convex hull
float box_dist = 1500;                          // distance in each dimension surrounding line segment for convex hull

// Path Planning Variables

float max_distance = 3;                                // Maximum distance allowed from path or endpoint
tuple<int, int> current_end_node;                      // Current end node
float max_angle = M_PI/4.0;                              // Maximum angle desired heading can take
bool ready_for_path = true;                            // Whether or not `calculate_deltas` node is ready for a new path
bool skip = false;                                     // Whether or not to skip current curve generation loop
tuple<float, float> des_gps;                           // Next desired GPS location
int node_size = 1;                                     // Size of node in graph in meters
unordered_set<int> blocked_nodes;                      // Nodes that should be avoided for path generation
unordered_set<int> center_blocked_nodes;               // The `anchor` blocked nodes, used only in generated the full `blocked_nodes` set
float padding = 1.5;                                   // Amount of padding to place around objects in meters
int padding_num = max((int) floor(padding / node_size),1); // Number of nodes around object to block out
float des_heading = M_PI/2.0;                                 // The desired heading relative to global heading
vector<float> start_xs, start_ys;
tuple<vector<float>, vector<float>> prev_path = {start_xs, start_ys};
int loaded_graph_size = 50;

// State Tracking Variables

float prev_heading = 0;                                    // Previous heading value
float delta_angle, delta_angle_cos, delta_angle_sin, dist; // Change in heading, trig values related to that change and change in position since last state
chrono::high_resolution_clock::time_point state_stop;      // Time just before a new state is to be computed
chrono::high_resolution_clock::time_point state_start;     // Time just after a new state has been computed
chrono::milliseconds duration;                             // Duration in milliseconds between state updates

// Returns distance in meters between two gps positions
float get_distance_between_gps(tuple<float, float> gps1, tuple<float, float> gps2){
    // Radius of the Earth in meters
    float radius_of_earth = 6371000;

    // Converts latitudes into radians
    float lat1_rad = get<0>(gps1) * M_PI/180;
    float lat2_rad = get<0>(gps2) * M_PI/180;

    // Gets difference in latitude and longitude and converts to radians
    float lat_diff_rad = (get<0>(gps2) - get<0>(gps1)) * M_PI/180;
    float lng_diff_rad = (get<1>(gps2) - get<1>(gps1)) * M_PI/180;

    // Fancy trig to calculate distance between the points
    float a = sin(lat_diff_rad/2) * sin(lat_diff_rad/2) + cos(lat1_rad) * cos(lat2_rad) * sin(lng_diff_rad/2) * sin(lng_diff_rad/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return radius_of_earth * c;
}

// Calculates angle between two longitude/latitude pairs
// NOTE: This only works for two points relatively close together, should be fine for our use case
float get_angle_from_gps(tuple<float, float> gps1, tuple<float, float> gps2){
    // Gets difference in latitude
    float lat_delta = get<0>(gps2) - get<0>(gps1);
    // Gets difference in longitude
    float lng_delta = cosf(M_PI/180*get<0>(gps1))*(get<1>(gps2) - get<1>(gps1));
    return atan2f(lat_delta, lng_delta);
}

void update_bike_pos(tuple<float, float> prev_gps, tuple<float, float> new_gps){
    // Calculates change in position
    dist = get_distance_between_gps(prev_gps, new_gps);
    if(get<0>(prev_gps) != 0){
    	bike_pos = make_tuple(get<0>(bike_pos) + dist * cosf(data[7]), get<1>(bike_pos) + dist * sinf(data[7]));
    }
}

// Synchronize current heading with GPS heading
void synchronize_heading(){
    // Collects latest GPS data
    ros::spinOnce();

    // Averages current GPS position from 20 sample points
    tuple<float, float> prev_gps = cur_gps;
    float latitude_sum = 0;
    float longitude_sum = 0;
    for(int i = 0; i < 20; i++){
        while(cur_gps == prev_gps){
            ros::spinOnce();
        }
        // Adds sampled longitude and latitude to running sum
        latitude_sum += get<0>(cur_gps);
        longitude_sum += get<1>(cur_gps);
    }
    // Divides the running sum by the number of sampled points to get mean
    tuple<float, float> mean_cur_gps = make_tuple(latitude_sum/20, longitude_sum/20);

    // We store our current gps position for later use
    prev_gps = cur_gps;

    // Bike moves 1 meter per second for 4 seconds
    //my_serial.write("t1,4000");

    // Waits until the bike is done moving
    while(data[5] > 0){
        ros::spinOnce();
    }

    // Collects new GPS data
    ros::spinOnce();

    // Gets average GPS position after movement using 20 samples
    prev_gps = cur_gps;
    latitude_sum = 0;
    longitude_sum = 0;
    for(int i = 0; i < 20; i++){
        // Waits for new GPS data to come in if it has not
        while(cur_gps == prev_gps){
            ros::spinOnce();
        }
        // Adds longitude and latitude value to running sum
        latitude_sum += get<0>(cur_gps);
        longitude_sum += get<1>(cur_gps);
    }
    // Divides running sums by number of points sampled to get mean
    tuple<float, float> mean_after_gps = make_tuple(latitude_sum/20, longitude_sum/20);

    // Calculate angle between both mean GPS coords
    sync_head_amt = get_angle_from_gps(mean_cur_gps, mean_after_gps);

    // Updates heading value
    data[7] = data[7] + sync_head_amt;
}

// Callback function for the `frame_ready` topic that sets the ready variable
void update_ready(const std_msgs::Empty msg){
    ready = true;
}

// Callback function for the `ready_for_path` topic waiting for message from `calculate_deltas` node
void update_ready_for_path(const std_msgs::Empty msg){
    ready_for_path = true;
}

// Uses the Cantor mapping function that maps a point in R2 to a unique value in R1 (for hashing)
int cantor(tuple<int, int> node){
    int a = get<0>(node);
    int b = get<1>(node);

    int A = a >= 0 ? 2 * a : -2 * a - 1;
    int B = b >= 0 ? 2 * b : -2 * b - 1;
    int C = (A >= B ? A * A + A + B : A + B * B) / 2;
    return a < 0 && b < 0 || a >= 0 && b >= 0 ? C : -C - 1;
}

// Calculates difference between two vectors in R2
tuple<float, float> get_change(tuple<float, float> p1, tuple<float, float> p2){
    return(make_tuple(get<0>(p2) - get<0>(p1), get<1>(p2) - get<1>(p1)));
}

// Resets variables for path planning
void reset_vars(){
    // The nodes that have been deemed blocked by an object
    blocked_nodes.clear();
    center_blocked_nodes.clear();
}

//Takes cartesian point and returns the corresponding path planning node
tuple<int, int> get_node_from_point(tuple<float, float> point){
    float x, y;
    tie(x, y) = point;
    return make_tuple((int) floor(x/node_size), (int) floor(y/node_size));
}

void get_blocked_nodes(tuple<float, float, float, float> obj){
    float x1, x2, y1, y2, delta_x, m;
    int x, y;
    tie(x1, x2, y1, y2) = obj;

    if(x2 < x1){
        float tmp = x1;
        x1 = x2;
        x2 = tmp;

        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    bool vert = (x2 == x1);

    if(!vert){
        m = (y2 - y1) / (x2 - x1);
        delta_x = (float) (node_size/(pow((1+pow(m, 2)),0.5)));
    } else{
        float tmp_max = max(y1, y2);
        float tmp_min = min(y1, y2);
        y1 = tmp_min;
        y2 = tmp_max;
    }

    tuple<float, float> cur_point = {x1, y1};

    while(((get<0>(cur_point) < x2) and !vert) or ((get<1>(cur_point) < y2) and vert)){
        tuple<int, int> node = get_node_from_point(make_tuple(get<0>(cur_point), get<1>(cur_point)));
        tie(x, y) = node;

        if(center_blocked_nodes.find(cantor(node)) == center_blocked_nodes.end()){
            // We blocked nodes around the point according to the value of `padding_num`
            for(int y_ind = -(padding_num); y_ind < (padding_num+1); y_ind++) {
                for (int x_ind = -(padding_num); x_ind < (padding_num + 1); x_ind++) {
                    blocked_nodes.emplace(cantor(make_tuple(x + x_ind, y + y_ind)));
                    //cout << "(" << to_string(x + x_ind) << ", " << to_string(y + y_ind) << ")" << endl;
                }
            }
            center_blocked_nodes.emplace(cantor(node));
        }

        if(!vert){
            cur_point = make_tuple(get<0>(cur_point) + delta_x, get<1>(cur_point) + (m * delta_x));
        } else{
            cur_point = make_tuple(get<0>(cur_point), get<1>(cur_point) + node_size);
        }
    }

    tuple<int ,int> node = get_node_from_point(make_tuple(x2, y2));
    if(center_blocked_nodes.find(cantor(node)) == center_blocked_nodes.end()){
        tie(x, y) = node;

        if(center_blocked_nodes.find(cantor(node)) == center_blocked_nodes.end()) {
            // We blocked nodes around the point according to the value of `padding_num`
            for (int y_ind = -(padding_num); y_ind < (padding_num + 1); y_ind++) {
                for (int x_ind = -(padding_num); x_ind < (padding_num + 1); x_ind++) {
                    blocked_nodes.emplace(cantor(make_tuple(x + x_ind, y + y_ind)));
                    //cout << "(" << to_string(x + x_ind) << ", " << to_string(y + y_ind) << ")" << endl;
                }
            }
            center_blocked_nodes.emplace(cantor(node));
        }
    }
}

tuple<int, int> get_end_node(tuple<float ,float> bike_pos, float desired_heading) {
    return get_node_from_point(make_tuple(
            get<0>(bike_pos) + (cos(desired_heading) * (float) loaded_graph_size),
            get<1>(bike_pos) + (sin(desired_heading) * (float) loaded_graph_size)
    ));
}

tuple<int, int> get_start_node(tuple<float, float> bike_pos, float cur_heading){
    float new_point_dist = 2 * node_size;

    float x = cos(cur_heading)*new_point_dist + get<0>(bike_pos);
    float y = sin(cur_heading)*new_point_dist + get<1>(bike_pos);

    return get_node_from_point(make_tuple(x, y));
}

float euc_dist(tuple<float, float> a, tuple<float, float> b){
    return (float) sqrt(pow((get<0>(a) - get<0>(b)),2) + pow((get<1>(a) - get<1>(b)),2));
}

struct greater_bfs_item : binary_function <const tuple<float, tuple<int, int>, tuple<vector<float>, vector<float>>>, const tuple<float, tuple<int, int>, tuple<vector<float>, vector<float>>>, bool> {
    bool operator()(const tuple<float, tuple<int, int>, tuple<vector<float>, vector<float>>>& item1, const tuple<float, tuple<int, int>, tuple<vector<float>, vector<float>>>& item2) const {
        return get<0>(item1) >= get<0>(item2);
    }
};

tuple<vector<float>, vector<float>> bfs_path(unordered_set<int> blocked_nodes, tuple<int, int> start_node, tuple<int, int> end_node, tuple<float, float> bike_pos){
    tuple<int, int> cur_node;
    vector<float> xs, ys, new_xs, new_ys;
    vector<tuple<int, int>> neighbors;
    float cur_dist, extra_dist, new_value;
    int diff_x, diff_y;

    unordered_set<int> node_set;
    node_set.emplace(cantor(start_node));

    vector<float> init_xs = {get<0>(bike_pos)/node_size, (float) get<0>(start_node)};
    vector<float> init_ys = {get<1>(bike_pos)/node_size, (float) get<1>(start_node)};

    tuple<float, tuple<int, int>, tuple<vector<float>, vector<float>>> start_item = {0, start_node, make_tuple(init_xs, init_ys)};

    priority_queue<tuple<float, tuple<int, int>, tuple<vector<float>, vector<float>>>, vector<tuple<float, tuple<int, int>, tuple<vector<float>, vector<float>>>>, greater_bfs_item> pr_q;

    pr_q.emplace(start_item);

    tuple<vector<float>, vector<float>> best_path = get<2>(start_item);
    float best_dist = euc_dist(start_node, end_node);
    unordered_set<int> visited;

    while(!pr_q.empty()){
        cur_dist = get<0>(pr_q.top());
        cur_node = get<1>(pr_q.top());

        //cout << "DEQUEUED: (" << to_string(get<0>(cur_node)) << ", " << to_string(get<1>(cur_node)) << ")" << endl;

        xs = get<0>(get<2>(pr_q.top()));
        ys = get<1>(get<2>(pr_q.top()));
        pr_q.pop();
        //cout << "PRIORITY QUEUE SIZE: " << to_string(pr_q.size()) << endl;
        node_set.erase(cantor(cur_node));

        if(cur_node == end_node){
            return make_tuple(xs, ys);
        }

        if(visited.find(cantor(cur_node)) != visited.end()){
            continue;
        }

        visited.emplace(cantor(cur_node));

        if((float) get<0>(cur_node) > xs[xs.size()-2]){
            diff_x = 1;
        } else if((float) get<0>(cur_node) < xs[xs.size()-2]){
            diff_x = -1;
        } else{
            diff_x = 0;
        }

        if((float) get<1>(cur_node) > ys[ys.size()-2]){
            diff_y = 1;
        } else if((float) get<1>(cur_node) < ys[ys.size()-2]){
            diff_y = -1;
        } else{
            diff_y = 0;
        }

        neighbors = {};

        if(diff_x != 0 && diff_y != 0){
            neighbors = {
                    make_tuple(get<0>(cur_node) + diff_x, get<1>(cur_node)),
                    make_tuple(get<0>(cur_node), get<1>(cur_node) + diff_y),
                    make_tuple(get<0>(cur_node) + diff_x, get<1>(cur_node) + diff_y)
            };
        } else if(diff_x != 0){
            for(int i = -1; i < 2; i++){
                neighbors.emplace_back(make_tuple(get<0>(cur_node) + diff_x, get<1>(cur_node) + i));
            }
        } else{
            for(int i = -1; i < 2; i++){
                neighbors.emplace_back(make_tuple(get<0>(cur_node) + i, get<1>(cur_node) + diff_y));
            }
        }

        //cout << "NEIGH SIZE: " << to_string(neighbors.size()) << endl;
        for(int j = 0; j < 3; j++){
            tuple<int, int> neigh = neighbors[j];
            if(blocked_nodes.find(cantor(neigh)) != blocked_nodes.end()){
                //cout << "   NEIGH: (" << to_string(get<0>(neigh)) << ", " << to_string(get<1>(neigh)) << ")" << endl;
                //cout << "        FOUND IN BLOCKED" << endl;
                continue;
            }

            if(visited.find(cantor(neigh)) != visited.end()){
                //cout << "   NEIGH: (" << to_string(get<0>(neigh)) << ", " << to_string(get<1>(neigh)) << ")" << endl;
                //cout << "        FOUND IN VISITED" << endl;
                continue;
            }

            if(node_set.find(cantor(neigh)) != node_set.end()){
                //cout << "   NEIGH: (" << to_string(get<0>(neigh)) << ", " << to_string(get<1>(neigh)) << ")" << endl;
                //cout << "        FOUND IN NODE SET" << endl;
                continue;
            }

            if(abs(get<0>(neigh) - get<0>(start_node)) > loaded_graph_size or abs(get<1>(neigh) - get<1>(start_node)) > loaded_graph_size){
                continue;
            }

            new_xs = xs;
            new_xs.emplace_back(get<0>(neigh));

            new_ys = ys;
            new_ys.emplace_back(get<1>(neigh));

            if(euc_dist(neigh, end_node) < best_dist){
                best_dist = euc_dist(neigh, end_node);
                best_path = make_tuple(new_xs, new_ys);
            }

            if(get<0>(neigh) != get<0>(cur_node) && get<1>(neigh) != get<1>(cur_node)){
                extra_dist = (float) sqrt(2);
            }else {
                extra_dist = 1;
            }

            new_value = cur_dist + 1.0*extra_dist + 1.2*euc_dist(neigh, end_node);

            //cout << "   NEIGH: (" << to_string(get<0>(neigh)) << ", " << to_string(get<1>(neigh)) << ") -- VALUE: " << to_string(new_value) << endl;

            node_set.emplace(cantor(neigh));
            tuple<float, tuple<int, int>, tuple<vector<float>, vector<float>>> new_item = {new_value, neigh, make_tuple(new_xs, new_ys)};
            pr_q.emplace(new_item);
            //cout << "NOW_TOP: " << to_string(pr_q.top()->value) << endl;
        }
    }
    if(euc_dist(make_tuple(get<0>(best_path).back(), get<1>(best_path).back()), bike_pos) < max_distance){
        cout << "BIKE MUST BE STOPPED" << endl;
        exit(1);
    }
    return best_path;
}

// Updates `desired_gps_pos` if either it has not been set or if the distance to the current desired position is less than 4 meters.
void update_desired_gps_pos(){
    if(!desired_gps_set || get_distance_between_gps(cur_gps, desired_gps_pos) < 4){
        desired_gps_cli.call(desired_gps_obj);
        desired_gps_pos = make_tuple(desired_gps_obj.response.latitude, desired_gps_obj.response.longitude);
    }
}

tuple<vector<float>, vector<float>> adjust_path_for_interp(tuple<vector<float>, vector<float>> cur_path, float cur_heading){
    vector<float> new_xs, new_ys;

    for(int i = 0; i < get<0>(cur_path).size(); i++){
        new_xs.emplace_back(get<0>(cur_path)[i] - get<0>(cur_path)[0]);
        new_ys.emplace_back(get<1>(cur_path)[i] - get<1>(cur_path)[0]);
    }

    for(int i = 0; i < get<0>(cur_path).size(); i++){
        new_xs[i] = new_xs[i] * cos(-cur_heading) - new_ys[i] * sin(-cur_heading);
        new_ys[i] = new_xs[i] * sin(-cur_heading) + new_ys[i] * cos(-cur_heading);
    }

    return make_tuple(new_xs, new_ys);
}

int binary_search(const vector<float>& values, float target){
    int low = 0;
    int high = (int) values.size() - 1;
    int mid;

    while(low <= high){
        mid = (low + high) / 2;
        if(values[mid] == target){
            return mid;
        } else if(values[mid] < target){
            low = mid + 1;
        } else{
            high = mid - 1;
        }
    }
    return low;
}


float calculate_distance_from_path(tuple<float, float> bike_pos){
    vector<float> xs, ys;
    tie(xs, ys) = prev_path;
    int ind = binary_search(xs, get<0>(bike_pos));
    return euc_dist(make_tuple(xs[ind], ys[ind]), bike_pos);
}

float calculate_distance_from_end_node(tuple<float, float> bike_pos){
    return euc_dist(bike_pos, make_tuple(((float) get<0>(current_end_node)) * node_size, ((float) get<1>(current_end_node)) * node_size));
}

bool check_for_objects_in_path(tuple<vector<float>, vector<float>> path){
    vector<float> xs, ys;
    tie(xs, ys) = path;
    tuple<int, int> node;

    for(int i = 0; i < xs.size(); i++){
        node = get_node_from_point(make_tuple(xs[i] / node_size, ys[i] / node_size));
        if(blocked_nodes.find(cantor(node)) != blocked_nodes.end()){
            return true;
        }
    }
    return false;
}

bool should_new_path_be_generated(tuple<float, float> bike_pos){
    if(get<0>(prev_path).empty()){
        return true;
    }

    if(check_for_objects_in_path(prev_path)){
        return true;
    }

    if(calculate_distance_from_path(bike_pos) > max_distance){
        return true;
    }

    if(calculate_distance_from_end_node(bike_pos) < max_distance){
        return true;
    }

    return false;
}

tuple<vector<float>, vector<float>> get_direct_heading_path(tuple<float, float> bike_pos, float cur_heading, float desired_heading){
    float new_point_dist = 2 * node_size;

    tuple<float, float> point = {cos(cur_heading) * new_point_dist + get<0>(bike_pos), sin(cur_heading) * new_point_dist + get<1>(bike_pos)};
    tuple<float, float> end_point = {cos(desired_heading) * 30 + get<0>(bike_pos), sin(desired_heading) * 30 + get<1>(bike_pos)};

    float new_angle = atan2(get<1>(end_point) - get<1>(point), get<0>(end_point) - get<0>(point));

    vector<float> s_xs = {get<0>(bike_pos), get<0>(point)};
    vector<float> s_ys = {get<1>(bike_pos), get<1>(point)};
    tuple<vector<float>, vector<float>> path = {s_xs, s_ys};

    while(get<0>(point) < get<0>(end_point)){
        point = make_tuple(get<0>(point) + cos(new_angle) * node_size, get<1>(point) + sin(new_angle) * node_size);
        get<0>(path).emplace_back(get<0>(point));
        get<1>(path).emplace_back(get<1>(point));
    }

    get<0>(path).emplace_back(get<0>(end_point));
    get<1>(path).emplace_back(get<1>(end_point));

    return path;
}

// Creates the curve around objects
void create_path(){

    for(tuple<float, float, float, float> obj : obj_lst){
        obj = make_tuple(get<0>(obj) / 1000, get<1>(obj) / 1000, get<2>(obj) / 1000, get<3>(obj) / 1000);
        get_blocked_nodes(obj);
    }

    float used_desired_heading = des_heading;
    if(abs(des_heading-data[7]) > max_angle){
        used_desired_heading = (des_heading / des_heading) * max_angle;
    }

    tuple<vector<float>, vector<float>> path = get_direct_heading_path(bike_pos, (float) data[7], used_desired_heading);
    bool use_direct_path = !check_for_objects_in_path(path);

    if(should_new_path_be_generated(bike_pos) || use_direct_path) {
        //cout << "Generating New Path" << endl;
        vector<float> xs, ys;
        tuple<int,int> start_node;
        if(use_direct_path){
            tie(xs, ys) = path;
            current_end_node = get_node_from_point(make_tuple(xs.back(), ys.back()));
        } else {
            start_node = get_start_node(bike_pos, data[7]);
            current_end_node = get_end_node(bike_pos, used_desired_heading);

            cout << "END_NODE: (" << to_string(get<0>(current_end_node)) << ", " << to_string(get<1>(current_end_node)) << ")" << endl;

            path = bfs_path(blocked_nodes, start_node, current_end_node, bike_pos);

	    if(get<0>(bike_pos) != get<0>(path)[0] && get<1>(bike_pos) != get<1>(path)[0]){
            	xs.emplace_back(get<0>(bike_pos));
            	ys.emplace_back(get<1>(bike_pos));
	    }

            for (int i = 0; i < get<0>(path).size(); i++) {
                xs.emplace_back(get<0>(path)[i] * node_size);
                ys.emplace_back(get<1>(path)[i] * node_size);
            }
        }
        prev_path = make_tuple(xs, ys);

        tuple<vector<float>, vector<float>> interp_path = adjust_path_for_interp(
                make_tuple(xs, ys), data[7]);
        
        calc_deltas_pub.path_x = get<0>(interp_path);
        calc_deltas_pub.path_y = get<1>(interp_path);

        calc_deltas.publish(calc_deltas_pub);
    }
    // auto end = chrono::high_resolution_clock::now();
    // auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
    // ROS_INFO_STREAM("PATH PLANNING IS TAKING: " << (float) duration.count() / 1000.0 << " SECONDS");
}

tuple<float, float> find_obj_center(tuple<float, float, float, float> obj){
    return make_tuple(get<0>(obj) + get<1>(obj) / 2, get<2>(obj) + get<3>(obj) / 2);
}

float get_new_obj_ang(tuple<float, float, float, float> obj){
    float x1, x2, y1, y2;
    tie(x1, x2, y1, y2) = obj;
    return atan2(y2 - y1, x2 - x1) + (float) data[7];
}

float get_obj_len(tuple<float, float, float, float> obj){
    float x1, x2, y1, y2;
    tie(x1, x2, y1, y2) = obj;
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

tuple<float, float, float, float> get_obj_pos(tuple<float, float, float, float> obj){
    float x1, x2, y1, y2, tx1, tx2, ty1, ty2;
    tie(x1, x2, y1, y2) = obj;
    tx1 = cos(data[7]) * x1 - sin(data[7]) * y1 + get<0>(bike_pos);
    tx2 = cos(data[7]) * x2 - sin(data[7]) * y2 + get<0>(bike_pos);
    ty1 = sin(data[7]) * x1 + cos(data[7]) * y1 + get<1>(bike_pos);
    ty2 = sin(data[7]) * x2 + cos(data[7]) * y2 + get<1>(bike_pos);
    return make_tuple(tx1, tx2, ty1, ty2);
}

// Defines a Graph that will be used for determining what smaller objects
// Are actually part of the same object
class Graph {
		int V;
		vector <vector<int>> adj;
	public:
		Graph(int);
		void addEdge(int, int);
		void dfs(vector<int> &temp, int vert, vector<bool> &visited);
		vector<vector<int>> connectedComps();
};

// Initializes a graph
Graph::Graph(int a) {
	V = a;
	adj.resize(a);
}

// Adds an edge to the graph
void Graph::addEdge(int a, int b) {
	adj[a].push_back(b);
	adj[b].push_back(a);
}

// Finds all of the objects that belong to the same larger object
void Graph::dfs(vector<int> &temp, int vert, vector<bool> &visited) {
	visited[vert] = true;
	temp.push_back(vert);
	for (int i = 0; i < adj[vert].size(); i++) {
		if (!visited[adj[vert][i]]) {
			dfs(temp, adj[vert][i], visited);
		}
	}
}

// Generates a vector of all larger objects
vector<vector<int>> Graph::connectedComps() {
	vector<bool> visited(V);
	vector<vector<int>> connected;
	for (int i = 0; i < V; i++) {
		if (!visited[i]) {
			vector<int> temp;
			dfs(temp, i, visited);
			connected.push_back(temp);
		}
	}
	return connected;
}

// Returns distance from a point to a line segment
float pointToSeg(int x, int z, tuple<float, float, float, float> seg) {
    float x1, x2, z1, z2;
    tie (x1, x2, z1, z2) = seg;
	float segx = x1 - x2;
	float segz = z1 - z2;
	float lpx = x1 - x;
	float lpz = z1 - z;
	float seg_len = sqrt(pow(x1 - x2, 2) + pow(z1 - z2, 2));
	if (seg_len == 0) {
		return sqrt(pow(x1 - lpx, 2) + pow(z1 - lpz, 2));
	}
	float t = segx / seg_len * lpx / seg_len + segz / seg_len * lpz / seg_len;
	if (t < 0) {
		t = 0;
	} else if (t > 0) {
		t = 1;
	}
	float nx = segx * t;
	float nz = segz * t;
	return sqrt(pow(nx - lpx, 2) + pow(nz - lpz, 2));
}

// Returns distance between two line segments
float segDist(tuple<float, float, float, float> seg1, tuple<float, float, float, float> seg2) {
    float seg1x1, seg1x2, seg1z1, seg1z2, seg2x1, seg2x2, seg2z1, seg2z2;
    tie (seg1x1, seg1x2, seg1z1, seg1z2) = seg1;
    tie (seg2x1, seg2x2, seg2z1, seg2z2) = seg2;
	float dist = pointToSeg(seg1x1, seg1z1, seg2);
	float t1 = pointToSeg(seg1x2, seg1z2, seg2);
	if (dist > t1) {
		dist = t1;
	}
	float t2 = pointToSeg(seg2x1, seg2z1, seg1);
	if (dist > t2) {
		dist = t2;
	}
	float t3 = pointToSeg(seg2x2, seg2z2, seg1);
	if (dist > t3) {
		dist = t3;
	}
	return dist;
}

// Finds the left-most point in a vector of points
int leftMost(vector<vector<float>> points) {
	int min_ind = 0;
	for (int i = 0; i < points.size(); i++) {
		if (points[i][0] < points[min_ind][0]) {
			min_ind = i;
		} else if (points[i][0] == points[min_ind][0]) {
			if (points[i][1] > points[min_ind][1]) {
				min_ind = i;
			}
		}
	}
	return min_ind;
}

// Sets the
int orientation(vector<float> p,vector<float> q,vector<float> r){
	float val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
	if (val) {
		if (val > 0) {
			return 1;
		} else {
			return 2;
		}
	} else {
		return 0;
	}
}

// Generates line segments representing the convex hull of a group of objects
// This is used to consolidate the smaller objects that were deemed part
// of the same larger object
vector<tuple<float, float, float, float>> convHull(vector<tuple<float, float, float, float>> objects) {
	vector<vector<float>> points;
	float x1, x2, z1, z2;
	for (tuple<float, float, float, float> o : objects) {
	    tie (x1, x2, z1, z2) = o;
		points.push_back({x1, z1});
		points.push_back({x2, z2});
	}
	int n = points.size();
	if (n < 3) {
		return objects;
	}
	int l = leftMost(points);
	vector<int> hull;
	int p = l;
	int q;

	while(true) {
		hull.push_back(p);
		q = (p+1) % n;
		for (int i = 0; i < n; i++) {
			if (orientation(points[p], points[i], points[q]) == 2) {
				q = i;
			}
		}
		p = q;
		if (p == l) {
			break;
		}
	}
	vector<tuple<float, float, float, float>> new_objects;
	for (int i = 1; i < hull.size(); i++) {
		new_objects.push_back(make_tuple(points[hull[i-1]][0], points[hull[i]][0], points[hull[i-1]][1], points[hull[i]][1]));
	}
	new_objects.push_back(make_tuple(points[hull[hull.size()-1]][0], points[hull[0]][0], points[hull[hull.size()-1]][1], points[hull[0]][1]));
	return new_objects;
}


// Difference between points used to calculate intersections
vector<float> diff (vector<float> p1, vector<float> p2) {
    vector<float> vals;
    vals.push_back(p1[1] - p2[1]);
    vals.push_back(p2[0] - p1[0]);
    vals.push_back(-1 * (p1[0] * p2[1] - p2[0] * p1[1]));
    return vals;
}

// Rotates a point by theta
vector<float> rotatePoint(vector<float> point, float theta) {
    vector<float> new_point = {0, 0};
    new_point[0] = point[0] * cos(theta) - point[1] * sin(theta);
    new_point[1] = point[0] * sin(theta) + point[1] * cos(theta);
    return new_point;
}

// Translates a point according to some translation matrix
vector<float> transPoint(vector<float> point, vector<float> trans) {
    point[0] += trans[0];
    point[1] += trans[1];
    return point;
}

// Gets the bounding box for a line segment (the line segments for the
// bounding box are `box-dist` away from the actual line segment)
vector<vector<float>> getBoundingBox(float x1, float x2, float z1, float z2) {
    vector<float> mov = {x2, z2};
    vector<float> o1 = {x1 - mov[0], z1 - mov[1]};
    // Rotates o1 about o2/origin so that o1 is directly above o2
    float theta;
    if (o1[1] == 0) {
        if (o1[0] > 0) {
            theta = M_PI_2;
        } else {
            theta = -M_PI_2;
        }
    } else {
        theta = atan(o1[0]/ o1[1]);
        if (o1[0] < 0) {
            theta += M_PI;
        }
    }
    o1 = rotatePoint(o1, theta);
    vector<float> p1 = {-box_dist, o1[1] + box_dist};
    vector<float> p2 = {box_dist, o1[1] + box_dist};
    vector<float> p3 = {box_dist, -box_dist};
    vector<float> p4 = {-box_dist, -box_dist};
    //Rotates p1, p2, and p3 by negative theta (original orientation)

    p1 = rotatePoint(p1, -theta);
    p2 = rotatePoint(p2, -theta);
    p3 = rotatePoint(p3, -theta);
    p4 = rotatePoint(p4, -theta);

    // Translates points back to relative positions
    p1 = transPoint(p1, mov);
    p2 = transPoint(p2, mov);
    p3 = transPoint(p3, mov);
    p4 = transPoint(p4, mov);

    vector<vector<float>> box;
    box.push_back(p1);
    box.push_back(p2);
    box.push_back(p3);
    box.push_back(p4);
    return box;
}

// Determines whether a new object intersects any existing objects
bool intersection(float x1, float x2, float z1, float z2) {
    vector<vector<float>> points;
    float x3, x4, z3, z4;
    if (x1 < x2) {
        points = getBoundingBox(x1, x2, z1, z2);
    } else if (x1 > x2){
        points = getBoundingBox(x2, x1, z2, z1);
    } else if (z1 < z2) {
        points = {{x1 - box_dist, z1 - box_dist},
                  {x1 + box_dist, z1 - box_dist},
                  {x2 + box_dist, z2 + box_dist},
                  {x2 - box_dist, z2 + box_dist}};
    } else {
        points = {{x2 - box_dist, z2 - box_dist},
                  {x2 + box_dist, z2 - box_dist},
                  {x1 + box_dist, z1 + box_dist},
                  {x1 - box_dist, z1 + box_dist}};
    }
    vector<float> p2p3 = {points[2][0] - points[1][0], points[2][1] - points[1][1]};
    vector<float> p2p1 = {points[0][0] - points[1][0], points[0][1] - points[1][1]};
    float p2p3Dot = p2p3[0] * p2p3[0] + p2p3[1] * p2p3[1];
    float p2p1Dot = p2p1[0] * p2p1[0] + p2p1[1] * p2p1[1];
    vector<vector<float>> point_diffs = {diff(points[1], points[0]), diff(points[2], points[1]),
                                         diff(points[3], points[2]), diff(points[0], points[3])};
    vector<vector<float>> box_constraints = {{min(points[0][0], points[1][0]), max(points[0][0], points[1][0]),
                                                     min(points[0][1], points[1][1]), max(points[0][1], points[1][1])},
                                             {min(points[1][0], points[2][0]), max(points[1][0], points[2][0]),
                                                     min(points[1][1], points[2][1]), max(points[1][1], points[2][1])},
                                             {min(points[2][0], points[3][0]), max(points[2][0], points[3][0]),
                                                     min(points[2][1], points[3][1]), max(points[2][1], points[3][1])},
                                             {min(points[3][0], points[0][0]), max(points[3][0], points[0][0]),
                                                     min(points[3][1], points[0][1]), max(points[3][1], points[0][1])}};
    for (tuple<float, float, float, float> o : obj_lst) {
        tie (x3, x4, z3, z4) = o;
        vector<float> p2m = {x3 - points[1][0], z3 - points[1][1]};
        vector<float> p2m2 = {x4 - points[1][0], z4 - points[1][1]};
        float p2m_p2p3 = (p2m[0] * p2p3[0] +  p2m[1] * p2p3[1]);
        float p2m_p2p1 = (p2m[0] * p2p1[0] +  p2m[1] * p2p1[1]);
        float p2m2_p2p3 = (p2m2[0] * p2p3[0] +  p2m2[1] * p2p3[1]);
        float p2m2_p2p1 = (p2m2[0] * p2p1[0] +  p2m2[1] * p2p1[1]);

        if ((0 <= p2m_p2p3 && p2m_p2p3 < p2p3Dot && 0 <= p2m_p2p1 && p2m_p2p1 < p2p1Dot) ||
            (0 <= p2m2_p2p3 && p2m2_p2p3 < p2p3Dot && 0 <= p2m2_p2p1 && p2m2_p2p1 < p2p1Dot)) {
            return true;
        }
        if (x3 > x2) {
            float temp = x3;
            x3 = x4;
            x4 = temp;
            temp = z3;
            z3 = z4;
            z4 = temp;
        }
        vector<float> diffs_obj = {z3 - z4, x4 - x3, -1 * (x3 * z4 - x4 * z3)};
        if (z3 > z4) {
            float temp = z3;
            z3 = z4;
            z4 = temp;
        }
        for (int i = 0; i < 4; i++) {
            float D = point_diffs[i][0] * diffs_obj[1] - point_diffs[i][1] * diffs_obj[0];
            if (D != 0) {
                float x = (point_diffs[i][2] * diffs_obj[1] - point_diffs[i][1] * diffs_obj[2]) / D;
                float z = (point_diffs[i][0] * diffs_obj[2] - point_diffs[i][2] * diffs_obj[0]) / D;
                if (x3 <= x && x <= x4 && z3 <= z && z <= z4 && box_constraints[i][0] <= x &&
                    x <= box_constraints[i][1] && box_constraints[i][2] <= z && z <= box_constraints[i][3]) {
                    return true;
                }
            }
        }
    }
    return false;
}

// Condenses a group of newly detected objects. This means that any objects
// That intersected are destroyed and any new objects are coalesed with
// nearby objects
vector<tuple<float, float, float, float>> condenseObjects(vector<tuple<float, float, float, float>> objects) {
	Graph gr = Graph(objects.size());
	for (int x = 0; x < objects.size(); x++) {
		for (int y = x+1; y < objects.size(); y++) {
			if (segDist(objects[x], objects[y]) < group_dist) {
				gr.addEdge(x,y);
			}
		}
	}
	vector<vector<int>> groups = gr.connectedComps();
	vector<vector<tuple<float, float, float, float>>> grouped_objs;
	for (int i = 0; i < groups.size(); i++) {
		vector<tuple<float, float, float, float>> temp;
		for (int y = 0; y < groups[i].size(); y++) {
			temp.push_back(objects[groups[i][y]]);
		}
		grouped_objs.push_back(temp);
	}
	vector<tuple<float, float, float, float>> new_objects;
	for (int i = 0; i < grouped_objs.size(); i++) {
		vector<tuple<float, float, float, float>> temp = convHull(grouped_objs[i]);
		new_objects.insert(new_objects.end(), temp.begin(), temp.end());
	}
	return new_objects;
}

// Detects new objects from the latest LiDAR data
void object_detection() {
    //auto start = chrono::high_resolution_clock::now();
	vector<vector<float>> cells(cell_row, vector<float>(cell_col, 0));
	for (int i = 0; i < lvx_points.size(); i++) {
		float z = get<2>(lvx_points[i]);

		if(z < 1000){
		    continue;
		}

		int x = (get<0>(lvx_points[i]) + (width / 2)) / cell_dim;
		int y = (get<1>(lvx_points[i]) + (height / 2)) / cell_dim;
		if (cells[y][x] == 0 || z < cells[y][x]) {
			cells[y][x] = z;
		}
	}
	vector<float> close_vec(cell_col);
	for (int col = 0; col < cell_col; col++) {
		float prev = 0;
		float closest = max_dist;
		int counter = 0;
		float min_obj = 0;

		for (int row = 0; row < cell_row; row++) {
			if (cells[row][col] == 0) {
				counter = 0;
				min_obj = 0;
				continue;
			}
			if (counter == 0){
				min_obj = cells[row][col];
			} else{
				min_obj = min(min_obj, cells[row][col]);
			}
			if (abs(cells[row][col] - prev) > col_diff) {
				counter = 0;
				min_obj = 0;
			} else {
				counter += 1;
			}
			if (prev != 0 && prev > cells[row][col] + for_jump_diff) {
				closest = min(closest, cells[row][col]);
			}
			if (counter > counter_reps) {
				closest = min(closest, min_obj);
			}
            if (cells[row][col] != 0) {
                prev = cells[row][col];
            }
		}
		close_vec[col] = closest;
	}
	int left_bound = 0;
	int right_bound = 0;
	float x1, x2, z1, z2;
	float prev = max_dist;
	vector<tuple<float, float, float, float>> z_boys;

	for (int col = 0; col < cell_col; col++) {
	    z1 = left_bound * cell_dim - width / 2;
	    z2 = (right_bound + 1) * cell_dim - width / 2;
	    x1 = close_vec[left_bound];
	    x2 = close_vec[right_bound];
	    tie(x1, x2, z1, z2) = get_obj_pos(make_tuple(x1, x2, z1, z2));
		if (close_vec[col] < max_dist) {
			if (prev == max_dist) {
				left_bound = col;
				right_bound = col;
				prev = close_vec[col];
			} else if (same_obj_diff > abs(prev - close_vec[col])) {
				right_bound++;
				prev = close_vec[col];
			} else {
                if (not intersection(x1, x2, z1, z2)) {
                    z_boys.push_back(make_tuple(x1, x2, z1, z2));
                }
				prev = max_dist;
			}
		} else if (prev < max_dist) {
           if (not intersection(x1, x2, z1, z2)) {
                z_boys.push_back(make_tuple(x1, x2, z1, z2));
            }
			prev = max_dist;
		}
	}

    z1 = left_bound * cell_dim - width / 2;
    z2 = (right_bound + 1) * cell_dim - width / 2;
    x1 = close_vec[left_bound];
    x2 = close_vec[right_bound];

    tie(x1, x2, z1, z2) = get_obj_pos(make_tuple(x1, x2, z1, z2));

    if (prev < max_dist && not intersection(x1, x2, z1, z2)) {
        z_boys.push_back(make_tuple(x1, x2, z1, z2));
    }

	vector<tuple<float, float, float, float>> cond_objs = condenseObjects(z_boys);
    obj_lst.insert(obj_lst.end(), cond_objs.begin(), cond_objs.end());
        for(auto & i : obj_lst){
	    ROS_INFO_STREAM("OBJECT: (" << get<0>(i) << ", " << get<1>(i) << ", " << get<2>(i) << ", " << get<3>(i) << ")");
	}

	//auto end = chrono::high_resolution_clock::now();
	//auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
	//ROS_INFO_STREAM("OBJECT DETECTION TOOK: " << (float) duration.count()/1000.0 << " SECONDS");
}



// Converts angle to a unit direction vector
tuple<float, float> conv_ang_to_dir_vec(float ang){
    return make_tuple(cos(ang), sin(ang));
}

//ofstream o_file ("/home/ubuntu/Autocycle/BezierAutocycle/ros/bruh.txt", ios::out);

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

                        // Adjusts for roll
                        x = x*cos(-data[1]) - y*sin(-data[1]);
                        y = x*sin(-data[1]) + y*cos(-data[1]);

                        if(z !=0 && x > -half_width && x < half_width && y > -height_of_lidar && y < half_height){
                            lvx_points.push_back(make_tuple(x, y, z));
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
    lvx_points.shrink_to_fit();
    //o_file << endl;
}

// Converts an object to its string representation
string object_to_string(tuple<float, float, float, float> obj){
    return("(" + to_string(get<0>(obj)) + ", " + to_string(get<1>(obj)) + ", " + to_string(get<2>(obj)) + ", " + to_string(get<3>(obj)) + ")");
}

void record_output(){
  string time_string = to_string(chrono::duration_cast<chrono::nanoseconds>(chrono::system_clock::now().time_since_epoch()).count());
  output_file << time_string;
  output_file << "\n[";
  for(int i = 0; i < obj_lst.size(); i++){
      tuple<float, float, float, float> obj = obj_lst[i];
      output_file << object_to_string(obj);
      if(i != obj_lst.size() - 1){
          output_file << ", ";
      }
  }
  output_file << "]\n[";
  for(int i = 0; i < get<0>(prev_path).size(); i++){
      output_file << "(" + to_string(get<1>(prev_path)[i]) + ", " + to_string(get<0>(prev_path)[i]) + ")";
        if(i != get<0>(prev_path).size() - 1){
            output_file << ", ";
        }
  }
  output_file << "]\n";
  output_file << to_string(des_heading);
  output_file << "\n" << "(" << to_string(get<0>(bike_pos)) << ", " << to_string(get<1>(bike_pos)) << ")";
  output_file << "\n[";
  for(int i = 0; i < data.size(); i++){
    output_file << to_string(data[i]);
    if(i != data.size() - 1){
      output_file << ", ";
    }
  }
  output_file << "]\n";
}

void get_data(const autocycle_extras::Data new_data){
    is_new_data = true;
    data = new_data.data;
    data[7] = data[7] + sync_head_amt;
    if(get<0>(cur_gps) != 0){
        update_bike_pos(cur_gps, make_tuple(data[9], data[10]));
    }
    prev_gps = cur_gps;
    cur_gps = make_tuple(data[9], data[10]);
    // Writes tab separated float data to `tab_file` and new line
    for(int i = 0; i < data.size(); i++){
        tab_file << data[i] << "\t";
    }
    tab_file << endl;
    record_output();
    // tuple<float, float> vel_vec = conv_ang_to_dir_vec(data[7]) * data[5];
    // float omega_mag;
    // if(data[8] > 0){
    //     omega_mag = -data[8];
    // }
    // else{
    //     omega_mag =  data[8];
    // }

    // tuple<float, float> omega_cross_r = make_tuple(omega_mag*get<1>(r_hat), omega_mag*get<0>(r_hat))
    // data[5] = sqrt(pow(get<0>(vel_vec) + get<0>(omega_cross_r), 2) + pow(get<1>(vel_vec) + get<1>(omega_cross_r), 2));
}

// The main navigation loop
int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Initailizes the output file
  string time_string = to_string(chrono::duration_cast<chrono::nanoseconds>(chrono::system_clock::now().time_since_epoch()).count());
  output_file.open("/home/ubuntu/test_data/" + time_string + ".txt");
  tab_file.open("/home/ubuntu/text_data/tab/" + time_string + ".txt");

  // Creates subscriber for updating roll
  ros::Subscriber update_data = nh.subscribe("sensors/data", 1, &get_data);

  // Creates subscriber that waits for new lidar frame to be ready
  ros::Subscriber ready_sub = nh.subscribe("cycle/frame_ready", 1, &update_ready);

  // Creates subscriber that updates velocity
  ros::Subscriber ready_for_path_sub = nh.subscribe("cycle/ready_for_path", 1, &update_ready_for_path);

  // Synchronizes heading values with GPS heading
  //synchronize_heading();
  
  ros::service::waitForService("due_ready");
  ros::service::waitForService("calc_deltas");
  ros::service::waitForService("ready_to_start");
  // ros::service::waitForService("get_desired_gps");
  // Creates publisher for calculating new deltas
  calc_deltas = nh.advertise<autocycle_extras::CalcDeltas>("cycle/calc_deltas", 1);

  // Creates server client for getting subsequent desired GPS positions
  // desired_gps_cli = nh.serviceClient<autocycle_extras::DesiredGPS>("get_desired_gps");

  for(int i = 0; i < 16; i++){
      data.push_back(-1);
  }

  // Sets desired heading (for now the initial heading)
  while(data[7] == -1){
    cout << "FUCK" << endl;  
    ros::spinOnce();
  }

  // Reserves the requisite space for the vectors in use
  lvx_points.reserve(15000);

  // Starts the clock
  auto start = chrono::high_resolution_clock::now();

  // starts second clock used only for updating object positions
  state_start = std::chrono::high_resolution_clock::now();

  // Navigation loop
  while(ros::ok()){
    // Tells ROS to look for any callbacks waiting to run
    ros::spinOnce();

    // Clears all points in preparation for new Livox data
    lvx_points.clear();

    // Updates object positions

    if(is_new_data){
        is_new_data = false;
        // Ends clock for updating object positions
        state_stop = std::chrono::high_resolution_clock::now();

        // Updates the duration since the last `state-stop`
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(state_stop - state_start);

        // Resets the clock for updating object positions
        state_start = std::chrono::high_resolution_clock::now();
    }

    // Only runs parse_lvx, fix_roll, and object detection if there is a new frame ready
    if(ready){
      // Ends the clock
      auto end = chrono::high_resolution_clock::now();

      // Gets the duration the clock was running
      auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);

      // Resets the clock
      start = chrono::high_resolution_clock::now();
      ROS_INFO_STREAM("NAV LOOP TOOK : " << (float) duration.count() / 1000.0 << " SECONDS");

      // Parses the LVX file
      parse_lvx();

      // Sets ready to false
      ready = false;

      // Clears f_done.lvx file while waiting for the rest of the loop to be ready
      f_done.open(path_to_lvx, ios::trunc);
      f_done.close();
      //ROS_INFO_STREAM("LVX file analyzed.");

      // ROS checks for call backs (To get the latest possible roll)
      ros::spinOnce();

      // Runs object detection on the new data
      object_detection();
    }

    //ROS_INFO_STREAM("Sending LiDAR data to Object Detection");
    // Once again checks for callbacks
    ros::spinOnce();

    // Generates a new path
    create_path();

  }

  // Clears the lvx file and shuts down. Mission Complete
  f_done.open(path_to_lvx, ios::trunc);
  f_done.write("done", 4);
  f_done.close();
  tab_file.close();
  //o_file.close();
  output_file.close();
  return 0;
}
