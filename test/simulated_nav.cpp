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

using namespace std;

// Creates serial object to write to

serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

// General use variable initialization

ofstream output_file;                                  // File to output data to
ofstream simulation_file;                              // File that will be human readable simulation results
string sim_input_path = "/home/ubuntu/sim_in.txt"      // Simulation input file
ifstream sim_in_file;
bool sim_running = true;
bool ready = false;                                    // True if new LiDAR frame is ready for analysis
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

bool ready_for_path = true;                            // Whether or not `calculate_deltas` node is ready for a new path
bool skip = false;                                     // Whether or not to skip current curve generation loop
tuple<float, float> des_gps;                           // Next desired GPS location
int path_width = 20;                                   // Width of graph in meters
int path_height = 20;                                  // Height of graph in meters
int node_size = 1;                                     // Size of node in graph in meters
int x_dim = path_width / node_size;                    // Width of graph in nodes
int y_dim = path_height / node_size;                   // Height of graph in nodes
unordered_set<int> blocked_nodes;                      // Nodes that should be avoided for path generation
unordered_set<int> center_blocked_nodes;               // The `anchor` blocked nodes, used only in generated the full `blocked_nodes` set
float padding = 1.5;                                   // Amount of padding to place around objects in meters
int padding_num = (int) (padding / (float) node_size); // Number of nodes around object to block out                    // The path generated
tuple<int, int> start_node = {y_dim/2, 0};             // Starting node for path
tuple<int, int> end_node;                              // End node for path
vector<float> ys;                                      // Path y values
vector<float> xs;                                      // Path x values
float des_heading = 0;                                 // The desired heading relative to global heading
float theta = 0;                                       // The desired heading relative to the current heading

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
    my_serial.write("t1,4000");

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
    int p1 = get<0>(node);
    int p2 = get<1>(node);
    return (((p1 + p2) * (p1 + p2 +1))/2) + p2;
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

    // The path being taken
    xs.clear();
    ys.clear();
}

//Takes cartesian point and returns the corresponding path planning node
tuple<int, int> get_node_from_point(tuple<float, float> point){
    float x = get<0>(point);
    float y = get<1>(point);
    return (make_tuple(
            (int) (((-y) + ((float) path_height / 2)) / (float) node_size),
            (int) (x / (float) node_size)
    ));
}

// Converts graph node into corresponding cartesian point
tuple<float, float> get_point_from_node(tuple<float, float> node){
    float y = get<0>(node);
    float x = get<1>(node);

    return(make_tuple(x * node_size, -((y  * node_size) - (path_height / 2))));
}

// Performs basic linear interpolation. This preprocessing can greatly improve the effectiveness of the cubic
// spline interpolation used to generate deltas
void augment_path(){
    // Initializes temporary vectors that will hold interpolated path
    vector<float> new_xs, new_ys;
    tuple<float, float> next_point, cur_point, change;

    // We start the path at the same point and set cur_point to that point
    new_xs.push_back(xs[0]);
    new_ys.push_back(ys[0]);
    cur_point = make_tuple(ys[0], xs[0]);

    for(int i=1;i<xs.size();i++){
        // We set next_point to the following point in our calculated path
        next_point = make_tuple(ys[i], xs[i]);
        // We calculate the difference between the points to get the midpoint
        change = get_change(cur_point, next_point);
        // We calculate the midpoint between `cur_point` and `next_point` and add this to our new path
        new_xs.push_back(get<1>(cur_point)+(get<1>(change)*0.5));
        new_ys.push_back(get<0>(cur_point)+(get<0>(change)*0.5));
        // We add the next point to the path and set that as our current point for the next iteration
        new_xs.push_back(get<1>(next_point));
        new_ys.push_back(get<0>(next_point));
        cur_point = next_point;
    }
    // We set the global `xs` and `ys` to this new interpolated path
    xs = new_xs;
    ys = new_ys;
}

// Returns the list of nodes that an object is blocking
void get_blocked_nodes(tuple<float, float, float, float> obj){
    float x1, x2, y1, y2, tmp, m;
    double delta_x;
    int x, y;
    tuple<int, int> node, new_node;
    // Sets x1, x2, y1, y2 to values corresponding to end points of the object
    tie (x1, x2, y1, y2) = obj;

    // For this to work we need the point (x1, y1) to have a lower x value than (x2, y2). If this is not already the
    // case, we swap the values of the points:
    if(x2 < x1){
        tmp = x1;
        x1 = x2;
        x2 = tmp;

        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    // We set a starting point at (x1, y1)
    double cur_point [2] = {x1, y1};

    // vert is true if the object is perfectly vertical. This is relevant to avoid divide by 0 errors
    bool vert = x2 - x1 == 0;

    // If object is not vertical, we calculate the slope of the object and set delta_x to the change in x value if we
    // were to move up the object a length equal to the size of our graph nodes.
    if(!vert){
        m = (y2 - y1) / (x2 - x1);
        delta_x = node_size/sqrt(1+pow(m, 2));
    }

    // The basic idea is that we are setting our current point to points on the object and adding the nodes around that
    // point to the blocked nodes set. If the object is vertical we continue until our cur point's y value exceeds the
    // second point's y value. If the object is not vertical we continue until our cur point's x value exceeds the second
    // point's x value.
    while((!vert && (cur_point[0] < x2)) || (vert && (cur_point[1] < y2))){
        // We get the node corresponding to the current node
        node = get_node_from_point(make_tuple(cur_point[0], cur_point[1]));
        // We check whether we've already used this node to generate blocked nodes
        if(center_blocked_nodes.find(cantor(node)) == center_blocked_nodes.end()){
            // We add this node to the list of nodes used to generate more blocked nodes
            center_blocked_nodes.insert(cantor(node));
            // We initialize a vector that will hold the nodes surrounding this center blocked node
            vector<tuple<int, int>> cur_blocked;
            x = get<1>(node);
            y = get<0>(node);

            // We blocked nodes around the point according to the value of `padding_num`
            for(int y_ind = -(padding_num); y_ind < (padding_num+1); y_ind++){
                for(int x_ind = -(padding_num); x_ind < (padding_num+1); x_ind++){
                    if(0 <= y+y_ind < y_dim && 0 <= x+x_ind < x_dim){
                        cur_blocked.emplace_back(y+y_ind, x+x_ind);
                    }
                }
            }
            // We calculate the cantor mapping of the blocked nodes and add them to our global list of blocked nodes
            for(auto & ind : cur_blocked){
                blocked_nodes.insert(cantor(ind));
            }
        }
        // We update the cur_point for the next iteration
        if(!vert){
            cur_point[0] = cur_point[0] + delta_x;
            cur_point[1] = cur_point[1] + (m * delta_x);
        } else {
            cur_point[1] = cur_point[1] + node_size;
        }
    }
    // The previous for loop works for the most part, but it is likely to skip over the other end point for our
    // object. We obviously need the nodes around the end point blocked off, so we run the same process here but just
    // for the end point.
    node = get_node_from_point(make_tuple(x2, y2));
    if(center_blocked_nodes.find(cantor(node)) == center_blocked_nodes.end()){
        vector<tuple<int, int>> cur_blocked;
        center_blocked_nodes.insert(cantor(node));
        x = get<1>(node);
        y = get<0>(node);

        for(int y_ind = -1; y_ind < 2; y_ind++){
            for(int x_ind = -1; x_ind < 2; x_ind++){
                if(0 <= y+y_ind < y_dim && 0 <= x+x_ind < x_dim){
                    cur_blocked.emplace_back(y+y_ind, x+x_ind);
                }
            }
        }
        for(auto & ind : cur_blocked){
            blocked_nodes.insert(cantor(ind));
        }
    }
}

// Checks whether a provided line intersects with the currently blocked off nodes. This is used to determine whether
// we can simply go towards the desired heading rather than worry about obstacles in our way.
bool line_intersect_object(tuple<tuple<int, int>, tuple<int, int>> end_points){
    // We extract points p1 and p2 from our input
    tuple<float, float> p1 = get_point_from_node(get<0>(end_points));
    tuple<float, float> p2 = get_point_from_node(get<1>(end_points));
    tuple<int, int> node;

    // We want the x value in p1 to come before the x value in p2. If this is not true, we swap the values
    if(get<0>(p2) < get<0>(p1)){
        tuple<float, float> tmp = p1;
        p1 = p2;
        p2 = tmp;
    }

    // We calculate the slope of our line. We should never have a divide by zero error since that would indicate we want
    // our bike to instantly turn 90 degrees. There are limits set up so that this never occurs.
    float m = (get<1>(p2) - get<1>(p1))/(get<0>(p2) - get<0>(p1));

    // We calculate the change in x and y if we move up the line by a length equal to our path planning node
    float delta_y = asin(m)*node_size;
    float delta_x = acos(m)*node_size;

    // We set our current point to p1
    float cur_x = get<0>(p1);
    float cur_y = get<1>(p1);

    // We move the current point up the line defined by the input, checking whether the current point lies on a blocked
    // node.
    while(cur_x < get<0>(p2)){
        // We convert the Cartesian current point to the path planning node it lies on
        node = get_node_from_point(make_tuple(cur_x, cur_y));
        // We check whether this node is blocked. If it is we immediately return true, indicated this path is blocked
        if(blocked_nodes.find(cantor(node)) != blocked_nodes.end()){
            return true;
        }
        // We update the current point
        cur_x += delta_x;
        cur_y += delta_y;
    }
    // The while loop above will likely skip over the final point. We check whether the associated node is blocked. If it
    // is we return true, indicating the path is blocked/
    node = get_node_from_point(p2);
    if(blocked_nodes.find(cantor(node)) != blocked_nodes.end()){
        return true;
    }
    // false is returned indicating the path is viable and not blocked!
    return false;
}

// Checks whether provided a desired heading, we can immediately attempt to actualize that heading
bool check_if_heading_path_available(){
    // These vectors will hold the path. The global vectors will only be updated with these values if the path is
    // deemed valid
    vector<float> temp_xs, temp_ys;
    tuple<float, float> first_point, last_point;

    // The path generated will be at a angle equal to `relative_heading`. This will idealy result in the bike going at
    // the desired heading.
    float relative_heading = des_heading - data[7];

    // The realtive heading is capped at +/- 70 degrees or about +/- 1.22 radians
    if(relative_heading > 1.22173){
        relative_heading = 1.221738;
    } else if(relative_heading < -1.22173){
        relative_heading = -1.22173;
    }

    // The path is initialized with nodes (0, 0) and (0, 1). If just (0, 0) is added the resulting interpolation
    // requires turns that are too sharp, adding this extra point smooths out the curve.
    temp_xs.push_back(get<1>(start_node));
    temp_xs.push_back(get<1>(start_node)+1);
    temp_ys.push_back(get<0>(start_node));
    temp_ys.push_back(get<0>(start_node));

    // The first point is eqivalent ot the second node added to the path above
    first_point = make_tuple(get<0>(start_node), get<1>(start_node)+1);
    // This last point is a point 30 meters away at the angle described by `relative_heading`
    last_point = make_tuple(sin(relative_heading)*30 + get<0>(start_node), cos(relative_heading)*30 + get<1>(start_node));

    // We add the values of this final point to the path
    temp_xs.push_back(get<1>(last_point));
    temp_ys.push_back(get<0>(last_point));

    // If the generated line does not intersect with objects, the points are converted into a format that can be easily
    // interpolated and added to the global xs and ys vector, then returns true
    if(!line_intersect_object(make_tuple(first_point, last_point))){
        for(int i = 0; i < temp_xs.size(); i++){
            xs.push_back(temp_xs[i]*node_size + node_size);
            ys.push_back(-temp_ys[i]*node_size + (path_height / 2));
        }
        return true;
    }
    // Returns false if the line generated intersected with a blocked node
    return false;
}

// Finds the shortest path from the starting node to the
// end node via a Breadth First Search(BFS)
void bfs(){
    // Instantiates a queue that will hold nodes to be processed by BFS
    deque<tuple<int, int>> q;
    // Keeps track of visited nodes so that we don't reprocess them
    unordered_set<int> visited;
    // Keeps track of parents of nodes so that we can generate the path by backtracking
    unordered_map<int, tuple<int, int>> parent;
    tuple<int, int> next_node, node;
    int x, y;
    // Keeps track of the total number of iterations. There seems to be something thats hanging that I've been unable
    // to fully diagnose so I've set a maximum iteration limit to deal with that if this is the source method
    int tot_count = 0;
    // We add the starting node to initiate BFS
    q.push_back(start_node);

    // We loop until either we've found the destination node or until the queue is empty
    while(!q.empty()){
        // If we exceed 2000 iterations we return immediately and skip this path planning session
        if(tot_count > 2000){
            skip = true;
            return;
        }
        // We access the next node from the queue
        next_node = q.front();

        // We increment the iteration counter and remove the first element from the queue we accessed above
        tot_count++;
        q.pop_front();

        // We unpack the x and y values from the node we popped
        y = get<0>(next_node);
        x = get<1>(next_node);

        // We create a tuple of tuples describing the 5 nodes adjacent and not behind the current node
        tuple<int, int> adj_nodes [5] = {make_tuple(y, x+1), make_tuple(y-1, x+1), make_tuple(y+1, x+1), make_tuple(y+1, x), make_tuple(y-1, x)};
        // We add the current node to the list of visited ndoes
        visited.insert(cantor(next_node));

        // We iterate over all adjacent nodes
        for(auto & node : adj_nodes){
            // If the node we are considering is out of bounds or if the node is blocked we ignore the node
            if(!(0 <= get<0>(node) && get<0>(node) <= y_dim) || !(0 <= get<1>(node) && get<1>(node) <= x_dim) || blocked_nodes.find(cantor(node)) != blocked_nodes.end()){
                continue;
            }

            // If the node has already been processed we ignore the node
            if(visited.find(cantor(node)) != visited.end()){
                continue;
            }

            // If the node has not already been invalidated we add it the queue to be processed later
            q.push_back(node);

            // We add this node to the visited list so that it is not processed as an adjacent node again
            visited.insert(cantor(node));

            // We set the parent of this node as the node that we popped from the queue
            parent[cantor(node)] = next_node;

            // If we found the end_node we clear the queue and break out of the loops
            if(node == end_node){
                q.clear();
                break;
            }
        }
    }

    // We perform
    node = end_node;
    while(node != start_node) {
        xs.push_back(get<1>(node)*node_size + node_size);
        ys.push_back(-get<0>(node)*node_size + (path_height / 2));
        try {
            node = parent.at(cantor(node));
        } catch (out_of_range const &) {
            ROS_INFO_STREAM("THE BIKE SHOULD BE STOPPED"); // TODO: stop the bike
            ros::shutdown();
            break;
        }
    }
    xs.push_back(get<1>(start_node)*node_size + node_size);
    ys.push_back(-get<0>(node)*node_size + (path_height / 2));
    reverse(xs.begin(), xs.end());
    reverse(ys.begin(), ys.end());
}

// Updates end_node to approximate the desired heading
void update_end_node() {
    double m = tan(theta);
    int half_path_height = path_height / 2;
    tuple<int, int> top = get_node_from_point(make_tuple(half_path_height / m, half_path_height));
    tuple<int, int> bot = get_node_from_point(make_tuple((-half_path_height) / m, -half_path_height));
    tuple<int, int> right = get_node_from_point(make_tuple(path_width, m * (path_width)));

    if (0 <= get<1>(top) && get<1>(top) < x_dim) {
        end_node = top;
    } else if (0 <= get<1>(bot) && get<1>(bot) < x_dim) {
        end_node = bot;
    } else {
        end_node = right;
    }
}

// Updates `desired_gps_pos` if either it has not been set or if the distance to the current desired position is less than 4 meters.
void update_desired_gps_pos(){
    if(!desired_gps_set || get_distance_between_gps(cur_gps, desired_gps_pos) < 4){
        desired_gps_cli.call(desired_gps_obj);
        desired_gps_pos = make_tuple(desired_gps_obj.response.latitude, desired_gps_obj.response.longitude);
    }
}

// Creates the curve around objects
void generate_curve() {
    ros::spinOnce();
    //update_desired_gps_pos();
    //des_heading = get_angle_from_gps(cur_gps, desired_gps_pos);
    ready_for_path = false;
    des_heading = 0;
    theta = des_heading - data[7];
    reset_vars();
    update_end_node();
    vector<tuple<float, float, float, float>> real_obj_lst;
    real_obj_lst.reserve(obj_lst.size());
    float x1, x2, z1, z2;

    for(auto & i : obj_lst){
        tie (x1, x2, z1, z2) = i;
        real_obj_lst.emplace_back(make_tuple(z1/1000, z2/1000, x1/1000, x2/1000));
    }

    for (auto & i : real_obj_lst) {
        get_blocked_nodes(i);
    }

    if(!check_if_heading_path_available()){
        bfs();
        if(skip){
            skip = false;
            return;
        }
    } else{
        augment_path();
    }

    calc_deltas_pub.path_x = xs;
    calc_deltas_pub.path_y = ys;

    calc_deltas.publish(calc_deltas_pub);


    // auto end = chrono::high_resolution_clock::now();
    // auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
    // ROS_INFO_STREAM("PATH PLANNING IS TAKING: " << (float) duration.count() / 1000.0 << " SECONDS");
}
// Updates the positions of previously found objects according to the
// telemetry from the bike
void update_object_positions(float delta_time){
    delta_angle = data[7] - prev_heading;
    prev_heading = data[7];
    new_obj_lst.clear();

    dist = data[5] * delta_time;

    float x1, x2, z1, z2, rx1, rx2, rz1, rz2;

    for(auto & i : obj_lst){
        tie (x1, x2, z1, z2) = i;
        rx1 = x1 - delta_angle * z1;
        rz1 = z1 - dist - delta_angle * x1;

        rx2 = x2 - delta_angle * z2;
        rz2 = z2 - dist - delta_angle * x2;
        if(rz1 < 0 && rz2 < 0){
	        continue;
        }
        new_obj_lst.emplace_back(make_tuple(rx1, rx2, rz1, rz2));
    }
    obj_lst.clear();
    obj_lst = new_obj_lst;
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

// Gets the latest gps Data
void get_gps(const autocycle_extras::GPS data){
    cur_gps = make_tuple(data.longitude, data.latitude);
}

void get_data(const autocycle_extras::Data new_data){
    data = new_data.data;
    data[7] = data[7] + sync_head_amt;
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

// Converts angle to a unit direction vector
tuple<float, float> conv_ang_to_dir_vec(float ang){
    return make_tuple(cos(ang), sin(ang));
}

// Converts an object to its string representation
string object_to_string(tuple<float, float, float, float> obj){
    return("(" + to_string(get<0>(obj)) + ", " + to_string(get<1>(obj)) + ", " + to_string(get<2>(obj)) + ", " + to_string(get<3>(obj)) + ")");
}

void record_output(){
  string time_string = to_string(chrono::duration_cast<chrono::nanoseconds>(chrono::system_clock::now().time_since_epoch()).count());
  output_file << time_string;
  output_file << "\n[";
  for(auto &i : obj_lst){
      output_file << object_to_string(i) + " ";
  }
  output_file << "]\n[";
  for(int i = 0; i < xs.size(); i++){
      output_file << "(" + to_string(ys[i]) + ", " + to_string(xs[i]) + ") ";
  }
  output_file << "]\n";
  output_file << to_string(des_heading);
  output_file << "\n[";
  for(int i = 0; i < data.size(); i++){
    output_file << to_string(data[i]) << " ";
  }
  output_file << "]\n";
}

void record_sim_output(){
  simulation_file << "\nOutput:\n\nObjects:\n"
  simulation_file << "\n[";
  for(auto &i : obj_lst){
      simulation_file << object_to_string(i) + " ";
  }
  simulation_file << "\nPath:\n\n"
  simulation_file << "]\n[";
  for(int i = 0; i < xs.size(); i++){
      simulation_file << "(" + to_string(ys[i]) + ", " + to_string(xs[i]) + ") ";
  }
  simulation_file << "]\n";
  simulation_file << "\nCollected Data:\n\n"
  simulation_file << "\n[";
  for(int i = 0; i < data.size(); i++){
    simulation_file << to_string(data[i]) << " ";
  }
  simulation_file << "]\n\n";
}

void read_next_sim_info(){

    simulation_file << "Input:\n\nObjects:\n";

    int parity = 0;
    string line, token;
    string delimiter = " ";
    float x1, x2, z1, z2;
    vector<float> new_data;

    for(int i = 0; i < 2; i++){
        if(i == 1){
            simulation_file << "\nNew Data:\n";
        }
        getline(sim_in_file, line);
        simulation_file << line;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            s.erase(0, pos + delimiter.length());
            switch(i){
                case 0: switch(parity){
                    case 0 : x1 = stof(token);
                             break;
                    case 1 : x2 = stof(token);
                             break;
                    case 2 : z1 = stof(token);
                             break;
                    case 3 : z2 = stof(token);
                             obj_lst.emplace_back(make_tuple(x1, x2, z1, z2));
                             break;
                }
                break;
                case 1: new_data.emplace_back(stof(token));
            }
        }
    }
    data = new_data;
}

// The main navigation loop
int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Initailizes the output file
  string time_string = to_string(chrono::duration_cast<chrono::nanoseconds>(chrono::system_clock::now().time_since_epoch()).count());
  mkdir("/home/ubuntu/simulation/" + time_string + "/");
  output_file.open("/home/ubuntu/simulation" + time_string + "/simulation_output.txt");
  simulation_file.open("/home/ubuntu/simulation" + time_string + "/simulation_human_readable_output.txt");

  sim_in_file.open(sim_input_path);

  // Creates subscriber for updating roll
  ros::Subscriber update_data = nh.subscribe("sensors/data", 1, &get_data);

  // Creates subscriber that waits for new lidar frame to be ready
  ros::Subscriber ready_sub = nh.subscribe("cycle/frame_ready", 1, &update_ready);

  // Creates subscriber that updates velocity
  ros::Subscriber ready_for_path_sub = nh.subscribe("cycle/ready_for_path", 1, &update_ready_for_path);

  ros::service::waitForService("calc_deltas");
  // ros::service::waitForService("get_desired_gps");

  // Creates publisher for calculating new deltas
  calc_deltas = nh.advertise<autocycle_extras::CalcDeltas>("cycle/calc_deltas", 1);

  // Creates server client for getting subsequent desired GPS positions
  // desired_gps_cli = nh.serviceClient<autocycle_extras::DesiredGPS>("get_desired_gps");

  // Sets desired heading (for now the initial heading)
  while(data[7] == -1){
      ros::spinOnce();
  }

  xs.reserve(400);
  ys.reserve(400);

  // Starts the clock
  auto start = chrono::high_resolution_clock::now();

  // starts second clock used only for updating object positions
  state_start = std::chrono::high_resolution_clock::now();

  // Navigation loop
  while(ros::ok() && sim_running){
    read_next_sim_info();

    // Updates object positions
    update_object_positions(((float) duration.count())/1000.0);

    // Only runs parse_lvx, fix_roll, and object detection if there is a new frame ready
    // Ends the clock
    auto end = chrono::high_resolution_clock::now();

    // Gets the duration the clock was running
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);

    // Resets the clock
    start = chrono::high_resolution_clock::now();
    ROS_INFO_STREAM("NAV LOOP TOOK : " << (float) duration.count() / 1000.0 << " SECONDS");

    // ROS checks for call backs (To get the latest possible roll)
    ros::spinOnce();

    // Adjusts for roll
    fix_roll();

    // Generates a new path
    generate_curve();

    // Records data to output file
    record_output();

    record_sim_output();

  }

  //o_file.close();
  output_file.close();
  return 0;
}
