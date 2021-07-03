#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <autocycle_extras/ObjectList.h>
#include <autocycle_extras/Object.h>
#include <autocycle_extras/Point.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <autocycle_extras/CalcDeltas.h>
#include <cmath>
#include <set>
#include <vector>
#include <functional>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <tuple>

using namespace std;

// ready is true if there is a new frame from the lidar to analyze
bool ready = false;

// Callback function for the `frame_ready` topic that sets the ready variable
void update_ready(const std_msgs::Empty msg){
    ready = true;
}

// Will contain the points found by the lidar
vector<autocycle_extras::Point> points;

// Will contain the objects after object clusters are combined by the convex
// hull function and after intersecting objects have been removed
vector<autocycle_extras::Object> cond_objs;

// Initializes the file variable that will read the latest data from the lidar
ofstream f_done;

// stores the latest roll data
float roll = 0;

int height = 2400;   // vertical height of detection window in millimeters
int width = 2000;    // horizontal width of detection window in millimeters
int cell_dim = 50;   // dimension of cells in millimeters (cells are squares)

int half_height = height/2; // Half of the above variable
int half_width = width/2;   // Half of the above variable

int cell_row = ceil((1.0 * height) / cell_dim); // Number of cells in a row
int cell_col = ceil((1.0 * width) / cell_dim);  // Number of cells in a column

// Stores the current list of objects
autocycle_extras::ObjectList obj_lst;

// Stores the new set of objects in a temporary vector
// after adjusting the current objects for the bikes
// change in position and heading
autocycle_extras::ObjectList new_obj_lst;

// Publisher that publishes the newest path to a topic that is read by
// the `get_deltas` node
ros::Publisher calc_deltas;

// The object that will be published by the above publisher
autocycle_extras::CalcDeltas to_pub;

// Tunable parameters to determine if something is an object.

// If the z value for 2 adjacent cells in a column differ by
// more than this variable, they are considered different objects
int col_diff = 50;

int for_jump_diff = col_diff * 1.5;      // Expected min difference between cells in a column to indicate a jump forward.
int counter_reps = 2;                    // Number of reps required to dictate it is an object.
int same_obj_diff = 150;                 // maximum diff between horizontal cells to be considered the same object
int group_dist = 1500;					 // max dist between adjacent objects for convex hull
float max_dist = 4000;
float box_dist = 1500;                   // distance in each dimension surrounding line segment

float prev_heading = 0;
float heading = 0;
float velocity = 0;
float delta_angle, c, s, dist;

// Size of plot
int path_width = 20;
int path_height = 20;

// Size of each node
int node_size = 1;


// The dimensions of the graph (how many nodes in each direction)
int x_dim = path_width / node_size;
int y_dim = path_height / node_size;

// The nodes that have been deemed blocked by an object
unordered_set<int> blocked_nodes;
unordered_set<int> center_blocked_nodes;

// The path being taken
vector<tuple<float, float>> path;

// Starting node
tuple<int, int> start_node;

// End node
tuple<int, int> end_node;

// x values and y values
vector<float> ys;
vector<float> xs;

// Calculate Padding
float padding = 1.5;
int padding_num = (int) (padding / (float) node_size);

// Theta to adjust heading by
float theta = 0;

// The desired Heading
float des_heading = 0;

// Variables used to time the computation time
chrono::high_resolution_clock::time_point state_stop;
chrono::high_resolution_clock::time_point state_start;
chrono::milliseconds duration;

// Path to the Data file
string path_to_lvx = "f_done.lvx";

string path_to_string(){
    string output = "";
    for(auto &t; path){
        string.append("(" + to_string(get<0>t + ", " + get<1>t + "), ");
    }
    return output;
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

// Generates intermediary points along the generated path to assist in interpolation
void augment_path(){
    vector<tuple<float, float>> new_path;
    tuple<float, float> change;
    float new_x, new_y = 0;
    unsigned long last_ind;

    new_path.reserve(path.size()*2);
    new_path.push_back(path[0]);
    xs.clear();
    ys.clear();

    for(int i = 1; i < path.size(); i++){
        last_ind = new_path.size()-1;
        change = get_change(new_path[last_ind], path[i]);
        new_x = get<1>(new_path[last_ind])+(get<0>(change)*0.5);
        new_y = get<0>(new_path[last_ind])+(get<1>(change)*0.5);
        new_path.emplace_back(new_y, new_x);
        new_path.push_back(path[i]);
        xs.push_back(new_x*node_size + node_size);
        ys.push_back(-new_y*node_size + (path_height / 2));
    }

    path = new_path;
}

// Resets variables between generated paths
void reset_vars(){
    // The nodes that have been deemed blocked by an object
    blocked_nodes.clear();
    center_blocked_nodes.clear();

    // The path being taken
    path.clear();
    path.reserve(400);
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
        center_blocked_nodes.insert(cantor(node));
        vector<tuple<int, int>> cur_blocked;
        x = get<1>(node);
        y = get<0>(node);

        for(int y_ind = -1; y_ind < 2; y_ind++){
            for(int x_ind = -1; x_ind < 2; x_ind++){
                if(0 <= y+y_ind < y_dim && 0 <= x+x_ind < x_dim){
                    cur_blocked.emplace_back(y+y_ind, x+x_ind);
                }
            }
        }
    }
}

// Finds the shortest path from the starting node to the
// end node via a Breadth First Search(BFS)
void bfs(){
    deque<tuple<int, int>> q;
    unordered_set<int> visited;
    unordered_map<int, tuple<int, int>> parent;
    tuple<int, int> next_node, node;
    int x, y;
    int tot_count = 0; // DELETE THIS
    q.push_back(start_node);

    while(!q.empty()){
        next_node = q.front();
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
        path.emplace_back(node);
        xs.push_back(get<1>(node)*node_size + node_size);
        ys.push_back(-get<0>(node)*node_size + (path_height / 2));
        try {
            node = parent.at(cantor(node));
        } catch (out_of_range const &) {
            ROS_INFO_STREAM("THE BIKE SHOULD BE STOPPED"); // TODO: stop the bike
            break;
        }
    }
    path.emplace_back(start_node);
    reverse(path.begin(),path.end());
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

// Creates the curve around objects
void generate_curve() {
    ros::spinOnce();
    theta = des_heading - heading;

    reset_vars();
    update_end_node();
    vector<tuple<float, float, float, float>> real_obj_lst;
    real_obj_lst.reserve(obj_lst.obj_lst.size());

    for(auto & i : obj_lst.obj_lst){
        real_obj_lst.emplace_back(make_tuple(i.z1/1000, i.z2/1000, i.x1/1000, i.x2/1000));
    }

    for (auto & i : real_obj_lst) {
        get_blocked_nodes(i);
    }
    bfs();
    to_pub.path_x = xs;
    to_pub.path_y = ys;

    calc_deltas.publish(to_pub);

    // auto end = chrono::high_resolution_clock::now();
    // auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
    // ROS_INFO_STREAM("PATH PLANNING IS TAKING: " << (float) duration.count() / 1000.0 << " SECONDS");
}


// The main navigation loop
int main(int argc, char **argv) {
  if(argc < 5){
      cout << "You fool. You need to give me parameters. (e.g. roslaunch test_obj_to_ard.launch args:='2 3 4 1')" << endl;
      return 1;
  }
  autocycle_extras::Object obj;
  obj.x1 =  stof(argv[1]);
  obj.x2 =  stof(argv[2]);
  obj.z1 =  stof(argv[3]);
  obj.z2 =  stof(argv[4]);

  obj_lst.obj_lst.emplace_back(obj);

  // Initializes the Node and registers it with the master.
  ros::init(argc, argv, "test_obj_to_ard");
  ros::NodeHandle nh;

  // Creates server proxy for calculating new deltas
  calc_deltas = nh.advertise<autocycle_extras::CalcDeltas>("cycle/path", 1);

  // Sets desired heading (for now the initial heading)
  while(heading == -1){
      ros::spinOnce();
  }

  // Sets initial desired heading
  des_heading = heading;


  // enerates a new path
  generate_curve();
    
  ROS_INFO_STREAM("PATH: " << path_to_string());

  return 0;
}
