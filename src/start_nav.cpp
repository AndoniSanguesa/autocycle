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

using namespace std;

// Creates serial object to write to

serial::Serial my_serial("/dev/ttyACM0", (long) 115200, serial::Timeout::simpleTimeout(0));

// General use variable initialization

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

float roll = 0;               // Latest roll value
float heading = 0;            // Latest heading value
float velocity = 0;           // Latest velocity value
tuple<float, float> cur_gps;  // Latest longitude and latitude

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
int counter_reps = 1;                           // Number of consecutive consistent column values needed to immediately determine that an object is present
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
    float radius_of_earth = 6371000;
    float lat1_rad = get<0>(gps1) * M_PI/180;
    float lat2_rad = get<0>(gps2) * M_PI/180;
    float lat_diff_rad = (get<0>(gps2) - get<0>(gps1)) * M_PI/180;
    float lng_diff_rad = (get<1>(gps2) - get<1>(gps1)) * M_PI/180;
    float a = sin(lat_diff_rad/2) * sin(lat_diff_rad/2) + cos(lat1_rad) * cos(lat2_rad) * sin(lng_diff_rad/2) * sin(lng_diff_rad/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return radius_of_earth * c;
}

// Calculates angle between two longitude/latitude pairs
float get_angle_from_gps(tuple<float, float> gps1, tuple<float, float> gps2){
    float dy = get<0>(gps2) - get<0>(gps1);
    float dx = cosf(M_PI/180*get<0>(gps1))*(get<1>(gps2) - get<1>(gps1));
    return atan2f(dy, dx);
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
        latitude_sum += get<0>(cur_gps);
        longitude_sum += get<1>(cur_gps);
    }
    tuple<float, float> mean_cur_gps = make_tuple(latitude_sum/20, longitude_sum/20);
    prev_gps = cur_gps;

    // Bike moves 1 meter per second for 4 seconds
    my_serial.write("t1,4000");

    // Waits until the bike is done moving
    while(velocity > 0){
        ros::spinOnce();
    }

    // Collects new GPS data
    ros::spinOnce();

    // Waits for new GPS data to come in if it has not
    while(prev_gps == cur_gps){
        ros::spinOnce();
    }

    // Gets average GPS position after movement using 20 samples
    prev_gps = cur_gps;
    latitude_sum = 0;
    longitude_sum = 0;
    for(int i = 0; i < 20; i++){
        while(cur_gps == prev_gps){
            ros::spinOnce();
        }
        latitude_sum += get<0>(cur_gps);
        longitude_sum += get<1>(cur_gps);
    }
    tuple<float, float> mean_after_gps = make_tuple(latitude_sum/20, longitude_sum/20);

    // Calculate angle between both GPS data points
    sync_head_amt = get_angle_from_gps(mean_cur_gps, mean_after_gps);

    // Updates heading value
    heading += sync_head_amt;
}

// Callback function for the `frame_ready` topic that sets the ready variable
void update_ready(const std_msgs::Empty msg){
    ready = true;
}

// Callback function for the `ready_for_path` topic waiting for message from `calculate_deltas` node
void update_ready_for_path(const std_msgs::Empty msg){
    ready_for_path = true;
}

// Maps a tuple to 2 integers to a unique integer (for hashing)
int cantor(tuple<int, int> node){
    int p1 = get<0>(node);
    int p2 = get<1>(node);
    return (((p1 + p2) * (p1 + p2 +1))/2) + p2;
}

// Calculates difference between nodes
tuple<float, float> get_change(tuple<float, float> p1, tuple<float, float> p2){
    return(make_tuple(get<0>(p2) - get<0>(p1), get<1>(p2) - get<1>(p1)));
}

// Resets variables between generated paths
void reset_vars(){
    // The nodes that have been deemed blocked by an object
    blocked_nodes.clear();
    center_blocked_nodes.clear();

    // The path being taken
    xs.clear();
    ys.clear();
}

//Takes cartesian point and returns the corresponding node
tuple<int, int> get_node_from_point(tuple<float, float> point){
    float x = get<0>(point);
    float y = get<1>(point);
    return (make_tuple(
            (int) (((-y) + ((float) path_height / 2)) / (float) node_size),
            (int) (x / (float) node_size)
    ));
}

void augment_path(){
    vector<float> new_xs, new_ys;
    tuple<float, float> next_point, cur_point, change;
    new_xs.push_back(xs[0]);
    new_ys.push_back(ys[0]);
    cur_point = make_tuple(ys[0], xs[0]);

    for(int i=1;i<xs.size();i++){
        next_point = make_tuple(ys[i], xs[i]);
        change = get_change(cur_point, next_point);
        new_xs.push_back(get<1>(cur_point)+(get<1>(change)*0.5));
        new_ys.push_back(get<0>(cur_point)+(get<0>(change)*0.5));
        new_xs.push_back(get<1>(next_point));
        new_ys.push_back(get<0>(next_point));
        cur_point = next_point;
    }
    xs = new_xs;
    ys = new_ys;
}

// Converts graph node into corresponding cartesian point
tuple<float, float> get_point_from_node(tuple<float, float> node){
    float y = get<0>(node);
    float x = get<1>(node);

    return(make_tuple(x * node_size, -((y  * node_size) - (path_height / 2))));
}

// Returns the list of nodes that an object is blocking
void get_blocked_nodes(tuple<float, float, float, float> obj){
    float x1, x2, y1, y2, tmp, m;
    double delta_x;
    int x, y;
    tuple<int, int> node, new_node;
    tie (x1, x2, y1, y2) = obj;

    if(x2 < x1){
        tmp = x1;
        x1 = x2;
        x2 = tmp;

        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    double cur_point [2] = {x1, y1};
    bool vert = x2 - x1 == 0;
    if(!vert){
        m = (y2 - y1) / (x2 - x1);
        delta_x = node_size/sqrt(1+pow(m, 2));
    }

    while((!vert && (cur_point[0] < x2)) || (vert && (cur_point[1] < y2))){
        node = get_node_from_point(make_tuple(cur_point[0], cur_point[1]));
        if(center_blocked_nodes.find(cantor(node)) == center_blocked_nodes.end()){
            center_blocked_nodes.insert(cantor(node));
            vector<tuple<int, int>> cur_blocked;
            x = get<1>(node);
            y = get<0>(node);

            for(int y_ind = -(padding_num); y_ind < (padding_num+1); y_ind++){
                for(int x_ind = -(padding_num); x_ind < (padding_num+1); x_ind++){
                    if(0 <= y+y_ind < y_dim && 0 <= x+x_ind < x_dim){
                        cur_blocked.emplace_back(y+y_ind, x+x_ind);
                    }
                }
            }
            for(auto & ind : cur_blocked){
                blocked_nodes.insert(cantor(ind));
            }
        }
        if(!vert){
            cur_point[0] = cur_point[0] + delta_x;
            cur_point[1] = cur_point[1] + (m * delta_x);
        } else {
            cur_point[1] = cur_point[1] + node_size;
        }
    }

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

bool line_intersect_object(tuple<tuple<int, int>, tuple<int, int>> end_points){
    tuple<float, float> p1 = get_point_from_node(get<0>(end_points));
    tuple<float, float> p2 = get_point_from_node(get<1>(end_points));
    tuple<int, int> node;

    if(get<0>(p2) < get<0>(p1)){
        tuple<float, float> tmp = p1;
        p1 = p2;
        p2 = tmp;
    }

    float m = (get<1>(p2) - get<1>(p1))/(get<0>(p2) - get<0>(p1));
    float delta_y = asin(m)*node_size;
    float delta_x = acos(m)*node_size;
    float cur_x = get<0>(p1);
    float cur_y = get<1>(p1);

    while(cur_x < get<0>(p2)){
        node = get_node_from_point(make_tuple(cur_x, cur_y));
        if(blocked_nodes.find(cantor(node)) != blocked_nodes.end()){
            return true;
        }
        cur_x += delta_x;
        cur_y += delta_y;
    }

    node = get_node_from_point(p2);
    if(blocked_nodes.find(cantor(node)) != blocked_nodes.end()){
        return true;
    }
    return false;
}

bool check_if_heading_path_available(){
    vector<float> temp_xs, temp_ys;
    tuple<float, float> first_point, last_point;
    float relative_heading = des_heading - heading;

    if(relative_heading > 1.22173){
        relative_heading = 1.221738;
    } else if(relative_heading < -1.22173){
        relative_heading = -1.22173;
    }

    temp_xs.push_back(get<1>(start_node));
    temp_xs.push_back(get<1>(start_node)+1);
    temp_ys.push_back(get<0>(start_node));
    temp_ys.push_back(get<0>(start_node));

    first_point = make_tuple(get<0>(start_node), get<1>(start_node)+1);
    last_point = make_tuple(sin(relative_heading)*30 + get<0>(start_node), cos(relative_heading)*30 + get<1>(start_node)+1);
    temp_xs.push_back(get<1>(last_point));
    temp_ys.push_back(get<0>(last_point));
    if(!line_intersect_object(make_tuple(first_point, last_point))){
        for(int i = 0; i < temp_xs.size(); i++){
            xs.push_back(temp_xs[i]*node_size + node_size);
            ys.push_back(-temp_ys[i]*node_size + (path_height / 2));
        }
        return true;
    }
    return false;
}

// Finds the shortest path from the starting node to the
// end node via a Breadth First Search(BFS)
void bfs(){
    deque<tuple<int, int>> q;
    unordered_set<int> visited;
    unordered_map<int, tuple<int, int>> parent;
    tuple<int, int> next_node, node;
    int x, y;
    int tot_count = 0;
    q.push_back(start_node);

    while(!q.empty()){
        if(tot_count > 2000){
            skip = true;
            return;
        }
        next_node = q.front();

        tot_count++;
        q.pop_front();

        y = get<0>(next_node);
        x = get<1>(next_node);

        tuple<int, int> adj_nodes [5] = {make_tuple(y, x+1), make_tuple(y-1, x+1), make_tuple(y+1, x+1), make_tuple(y+1, x), make_tuple(y-1, x)};
        visited.insert(cantor(next_node));

        for(auto & adj_node : adj_nodes){
            node = adj_node;

            if(!(0 <= get<0>(node) && get<0>(node) <= y_dim) || !(0 <= get<1>(node) && get<1>(node) <= x_dim) || blocked_nodes.find(cantor(node)) != blocked_nodes.end()){
                continue;
            }

            if(visited.find(cantor(node)) != visited.end()){
                continue;
            }

            q.push_back(node);
            visited.insert(cantor(node));
            parent[cantor(node)] = next_node;

            if(node == end_node){
                q.clear();
                break;
            }
        }
    }

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
    theta = des_heading - heading;
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
    delta_angle = heading - prev_heading;
    prev_heading = heading;
    new_obj_lst.clear();

    dist = velocity * delta_time;

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
vector<tuple<float, float, float, float>> convHull(vector<tuple<float, float>> points) {
	vector<vector<float>> new_points;
    vector<tuple<float, float, float, float>> new_objects;
	float x, z;
	for (tuple<float, float> p : points) {
	    tie (x, z) = p;
		points.push_back({x, z});
	}
	int n = points.size();

    if (n == 1) {
        new_objects.emplace_back(make_tuple(new_points[0][0], new_points[0][0], new_points[0][1], new_points[0][1]));
        return new_objects;
    }
    if (n == 2) {
        new_objects.emplace_back(make_tuple(new_points[0][0], new_points[1][0], new_points[0][1], new_points[1][1]));
        return new_objects;
    }

	int l = leftMost(new_points);
	vector<int> hull;
	int p = l;
	int q;

	while(true) {
		hull.push_back(p);
		q = (p+1) % n;
		for (int i = 0; i < n; i++) {
			if (orientation(new_points[p], new_points[i], new_points[q]) == 2) {
				q = i;
			}
		}
		p = q;
		if (p == l) {
			break;
		}
	}
    float dist = 0;
    float angle = 0;
	for (int i = 1; i < hull.size(); i++) { 
        vector<float> curr_point = new_points[hull[i-1]];
        dist = sqrt(pow(curr_point[0] - new_points[hull[i]][0], 2) +  pow(curr_point[1] - new_points[hull[i]][1], 2));
        angle = atan2(curr_point[1] - new_points[hull[i]][1], curr_point[0] - new_points[hull[i]][0]);
        float z_comp = sin(angle) * same_obj_diff;
        float x_comp = cos(angle) * same_obj_diff;
        while (dist > same_obj_diff) {
            float temp_x = curr_point[0] + x_comp;
            float temp_z = curr_point[1] + z_comp;
            new_objects.emplace_back(make_tuple(curr_point[0], temp_x, curr_point[1], temp_z);
            curr_point[0] = temp_x;
            curr_point[1] = temp_z;
            dist = sqrt(pow(curr_point[0] - new_points[hull[i]][0], 2) +  pow(curr_point[1]- new_points[hull[i]][1], 2));
        }
        new_objects.emplace_back(make_tuple(curr_point[0], new_points[hull[i]][0], curr_point[1], new_points[hull[i]][1]));

	}
	vector<float> curr_point = new_points[hull[hull.size()-1]];
    dist = sqrt(pow(curr_point[0] - new_points[hull[0]][0], 2) +  pow(curr_point[1] - new_points[hull[0]][1], 2));
    angle = atan2(curr_point[1] - new_points[hull[0]][1], curr_point[0] - new_points[hull[0]][0]);
    float z_comp = sin(angle) * same_obj_diff;
    float x_comp = cos(angle) * same_obj_diff;
    while (dist > same_obj_diff) {
        float temp_x = curr_point[0] + x_comp;
        float temp_z = curr_point[1] + z_comp;
        new_objects.emplace_back(make_tuple(curr_point[0], temp_x, curr_point[1], temp_z);
        curr_point[0] = temp_x;
        curr_point[1] = temp_z;
        dist = sqrt(pow(curr_point[0] - new_points[hull[0]][0], 2) +  pow(curr_point[1]- new_points[hull[0]][1], 2));
    }
    new_objects.emplace_back(make_tuple(curr_point[0], new_points[hull[0]][0], curr_point[1], new_points[hull[0]][1]));
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


// Condenses a group of newly detected objects. This means that any objects
// That intersected are destroyed and any new objects are coalesed with
// nearby objects.
vector<tuple<float, float, float, float>> condenseObjects(vector<tuple<float,float>> points) {
	Graph gr = Graph(points.size());
    float dist = 0;
	for (int x = 0; x < points.size(); x++) {
		for (int y = x+1; y < points.size(); y++) {
            dist = sqrt(pow(get<0>(points[x]) - get<0>(points[y]), 2) + pow(get<1>(points[x]) - get<1>(points[y]), 2));
            if (dist < group_dist) {
				gr.addEdge(x,y);
			}
		}
	}
    // Find the connected points through depth first search where edges exist between points
    // if the distance is less than group_dist
	vector<vector<int>> groups = gr.connectedComps();
	vector<vector<tuple<float, float>>> grouped_points;
	for (int i = 0; i < groups.size(); i++) {
		vector<tuple<float, float>> temp;
		for (int y = 0; y < groups[i].size(); y++) {
			temp.push_back(points[groups[i][y]]);
		}
		grouped_points.push_back(temp);
	}
    // Find the convex hull of each grouping of points
	vector<tuple<float, float, float, float>> new_objects;
	for (int i = 0; i < grouped_points.size(); i++) {
		vector<tuple<float, float, float, float>> temp = convHull(grouped_points[i]);
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
	vector<tuple<float, float>> keypoints;
	for (int col = 0; col < cell_col; col++) {
        float prev = 0;
        int start = 0;
        int end = 0;
        bool forward_jump = false;
        float x_val = col * cell_dim - width / 2;

        for (int row = 0; row < cell_row; row++) {
            // Checks if cells[row][col] == 0 aka if the cell did not get any points inside it.
            // If no points then check if there are keypoints, when end - start > counter_reps
            // or forward_jump is true. Points are added to keypoints and start, end, forward_jump
            // are reset.
            if (cells[row][col] == 0) {
                if (end - start > counter_reps || forward_jump) {
                    for (int x = start; x <= end; x++) {
                        keypoints.emplace_back((x_val, cells[x][col]));
                    }
                }
                start = row;
                end = row;
                forward_jump = false;
                continue;
            }
            // If distance between prev and current exceed col_diff, check if their are keypoints
            // to add if so add them. Then reset start, end, and forward jump. If it does not exceed,
            // then increment end.
            if (abs(cells[row][col] - prev) > col_diff) {
                if (end - start > counter_reps || forward_jump) {
                    for (int x = start; x <= end; x++) {
                        keypoints.emplace_back((x_val, cells[x][col]));
                    }
                }
                start = row;
                end = row;
                forward_jump = false;
            } else {
                end += 1;
            }
            // If distance between curr + for_jump_diff is closer than prev and prev != 0 then check
            // if end - start > counter reps or forward_jump then add keypoints and reset start, end
            // also set forward_jump to true.
            if (prev != 0 && prev > cells[row][col] + for_jump_diff) {
                if (end - start > counter_reps || forward_jump) {
                    for (int x = start; x <= end; x++) {
                        keypoints.emplace_back((x_val, cells[x][col]));
                    }
                }
                start = row;
                end = row;
                forward_jump = true;
			}
            if (cells[row][col] != 0) {
                prev = cells[row][col];
            }
        }
        // Check if the are more keypoints to add for the column if so add them.
        if (end - start > counter_reps || forward_jump) {
            for (int x = start; x <= end; x++) {
                keypoints.emplace_back((x_val, cells[x][col]));
            }
        }
    }
    
    for(auto & i : obj_lst){
        tie (x1, x2, z1, z2) = i;
        keypoints.emplace_back((x1, z1));
        keypoints.emplace_back((x2, z2));
    }

    // Call condenseObjects with an input of keypoints to condense the keypoints found to objects
	cond_objs = condenseObjects(keypoints);
    obj_lst = cond_objs;
    for(auto & i : obj_lst){
	    ROS_INFO_STREAM("OBJECT: (" << get<0>(i) << ", " << get<1>(i) << ", " << get<2>(i) << ", " << get<3>(i) << ")");
	}

	//auto end = chrono::high_resolution_clock::now();
	//auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
	//ROS_INFO_STREAM("OBJECT DETECTION TOOK: " << (float) duration.count()/1000.0 << " SECONDS");
}

// Gets the latest roll
void get_roll(const std_msgs::Float32 data){
    roll = data.data;
}

// Gets the latest gps Data
void get_gps(const autocycle_extras::GPS data){
    cur_gps = make_tuple(data.longitude, data.latitude);
}

// Gets the latest Heading
void get_heading(const std_msgs::Float32 data){
    heading = data.data + sync_head_amt;
    if(check_if_heading_path_available()){
        ROS_INFO_STREAM("HEADING: " << data.data << "RELATIVE_HEADING: " << des_heading - data.data);
    }
}

// Gets the latest velocity
void get_velocity(const std_msgs::Float32 data){
    velocity = data.data;
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
                        x = *((uint32_t *) buff);
                        // z val
                        file.read(buff, 4);
                        y = *((uint32_t *) buff);

                        //z = z*0.9994 - y*0.0349
                        //y = z*0.0349 + y*0.9994

                        // Ignores tag and reflexivity
                        file.ignore(2);

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

// Adjusts detected points for roll from the bike
void fix_roll(){
  tuple<float, float, float> np; // new point tuple
  float x, y, z;                 // Point position variables

  // Updates each point for the given roll
  for(int i=0;i<lvx_points.size();i++){
    tie (x, y, z) = lvx_points[i];
    lvx_points[i] = make_tuple(x*cos(roll) - y*sin(roll), x*sin(roll) + y*cos(roll), z);
  }
}


// The main navigation loop
int main(int argc, char **argv) {
  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "navigation_communicator");
  ros::NodeHandle nh;

  // Creates subscriber for updating roll
  ros::Subscriber update_roll = nh.subscribe("sensors/roll", 1, &get_roll);

  // Creates subscriber that updates heading
  ros::Subscriber head_sub = nh.subscribe("sensors/heading", 1, &get_heading);

  // Creates subscriber that updates GPS data
  // ros::Subscriber gps_sub = nh.subscribe("sensors/gps", 1, &get_gps);

  // Creates subscriber that updates velocity
  ros::Subscriber vel_sub = nh.subscribe("sensors/vel", 1, &get_velocity);

  // Creates subscriber that updates velocity
  ros::Subscriber ready_sub = nh.subscribe("cycle/frame_ready", 1, &update_ready);

  // Creates subscriber that updates velocity
  ros::Subscriber ready_for_path_sub = nh.subscribe("cycle/ready_for_path", 1, &update_ready_for_path);

  // Synchronizes heading values with GPS heading
  synchronize_heading();
  
  ros::service::waitForService("due_ready");
  ros::service::waitForService("calc_deltas");
  // ros::service::waitForService("get_desired_gps");

  // Creates publisher for calculating new deltas
  calc_deltas = nh.advertise<autocycle_extras::CalcDeltas>("cycle/calc_deltas", 1);

  // Creates server client for getting subsequent desired GPS positions
  // desired_gps_cli = nh.serviceClient<autocycle_extras::DesiredGPS>("get_desired_gps");

  // Sets desired heading (for now the initial heading)
  while(heading == -1){
      ros::spinOnce();
  }

  // Reserves the requisite space for the vectors in use
  lvx_points.reserve(15000);
  xs.reserve(400);
  ys.reserve(400);

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

      // Adjusts for roll
      fix_roll();

      //ROS_INFO_STREAM("Points have been adjusted for roll.");

      // Runs object detection on the new data
      object_detection();
    }

    //ROS_INFO_STREAM("Sending LiDAR data to Object Detection");
    // Once again checks for callbacks
    ros::spinOnce();

    // Ends clock for updating object positions
    state_stop = std::chrono::high_resolution_clock::now();

    // Updates the duration since the last `state-stop`
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(state_stop - state_start);

    // Resets the clock for updating object positions
    state_start = std::chrono::high_resolution_clock::now();

    // Updates object positions
    update_object_positions(((float) duration.count())/1000.0);

    // Generates a new path
    if(ready_for_path){
        generate_curve();
    }

  }

  // Clears the lvx file and shuts down. Mission Complete
  f_done.open(path_to_lvx, ios::trunc);
  f_done.write("done", 4);
  f_done.close();
  //o_file.close();
  return 0;
}
