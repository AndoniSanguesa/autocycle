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

// Updates the positions of previously found objects according to the
// telemetry from the bike
void update_object_positions(float delta_time){
    delta_angle = heading - prev_heading;
    prev_heading = heading;
    new_obj_lst.obj_lst.clear();

    c = cos(delta_angle);
    s = sin(delta_angle);
    dist = velocity * delta_time;

    for(auto & i : obj_lst.obj_lst){
    autocycle_extras::Object rotated;
        rotated.z1 = i.z1*c - i.x1*s;
        rotated.x1 = i.z1*s + i.x1*c - dist;
        rotated.z2 = i.z2*c - i.x2*s;
        rotated.x2 = i.z2*s + i.x2*c - dist;
        if(rotated.z1 < 0 && rotated.z2 < 0){
            ROS_INFO_STREAM("BRERUUERDSHGSJKDFHSDIEUR");
	    continue;
        }
        new_obj_lst.obj_lst.emplace_back(rotated);
    }
    obj_lst.obj_lst.clear();
    obj_lst.obj_lst = new_obj_lst.obj_lst;
}

// Generates an Autocycle Object from floats (this should really be removed)
autocycle_extras::Object get_object(float x1, float x2, float z1, float z2){
	autocycle_extras::Object obj;
	obj.x1 = x1;
	obj.x2 = x2;
	obj.z1 = z1;
	obj.z2 = z2;
	return obj;
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
float pointToSeg(int x, int z, autocycle_extras::Object seg) {
	float segx = seg.x1 - seg.x2;
	float segz = seg.z1 - seg.z2;
	float lpx = seg.x1 - x;
	float lpz = seg.z1 - z;
	float seg_len = sqrt(pow(seg.x1 - seg.x2, 2) + pow(seg.z1 - seg.z2, 2));
	if (seg_len == 0) {
		return sqrt(pow(seg.x1 - lpx, 2) + pow(seg.z1 - lpz, 2));
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
float segDist(autocycle_extras::Object seg1, autocycle_extras::Object seg2) {
	float dist = pointToSeg(seg1.x1, seg1.z1, seg2);
	float t1 = pointToSeg(seg1.x2, seg1.z2, seg2);
	if (dist > t1) {
		dist = t1;
	}
	float t2 = pointToSeg(seg2.x1, seg2.z1, seg1);
	if (dist > t2) {
		dist = t2;
	}
	float t3 = pointToSeg(seg2.x2, seg2.z2, seg1);
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
vector<autocycle_extras::Object> convHull(vector<autocycle_extras::Object> objects) {
	vector<vector<float>> points;
	for (int i = 0; i < objects.size(); i++) {
		points.push_back({objects[i].x1, objects[i].z1});
		points.push_back({objects[i].x2, objects[i].z2});
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
	vector<autocycle_extras::Object> new_objects;
	for (int i = 1; i < hull.size(); i++) {
		new_objects.push_back(get_object(points[hull[i-1]][0], points[hull[i]][0], points[hull[i-1]][1], points[hull[i]][1]));
	}
	new_objects.push_back(get_object(points[hull[hull.size()-1]][0], points[hull[0]][0], points[hull[hull.size()-1]][1], points[hull[0]][1]));
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
bool intersection(float x1, float x2, float z1, float z2, vector<autocycle_extras::Object> objects) {
    vector<vector<float>> points;
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
    for (autocycle_extras::Object o : objects) {
        float x3 = o.x1;
        float x4 = o.x2;
        float z3 = o.z1;
        float z4 = o.z2;
        vector<float> p2m = {o.x1 - points[1][0], o.z1 - points[1][1]};
        vector<float> p2m2 = {o.x2 - points[1][0], o.z2 - points[1][1]};
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
vector<autocycle_extras::Object> condenseObjects(vector<autocycle_extras::Object> objects) {
	Graph gr = Graph(objects.size());
	for (int x = 0; x < objects.size(); x++) {
		for (int y = x+1; y < objects.size(); y++) {
			if (segDist(objects[x], objects[y]) < group_dist) {
				gr.addEdge(x,y);
			}
		}
	}
	vector<vector<int>> groups = gr.connectedComps();
	vector<vector<autocycle_extras::Object>> grouped_objs;
	for (int i = 0; i < groups.size(); i++) {
		vector<autocycle_extras::Object> temp;
		for (int y = 0; y < groups[i].size(); y++) {
			temp.push_back(objects[groups[i][y]]);
		}
		grouped_objs.push_back(temp);
	}
	vector<autocycle_extras::Object> new_objects;
	for (int i = 0; i < grouped_objs.size(); i++) {
		vector<autocycle_extras::Object> temp = convHull(grouped_objs[i]);
		new_objects.insert(new_objects.end(), temp.begin(), temp.end());
	}
	return new_objects;
}

// Detects new objects from the latest LiDAR data
void object_detection() {
    //auto start = chrono::high_resolution_clock::now();
	obj_lst.obj_lst.clear();
	vector<vector<float>> cells(cell_row, vector<float>(cell_col, 0));
	for (int i = 0; i < points.size(); i++) {
		float z = points[i].z;
		int x = (points[i].x + (width / 2)) / cell_dim;
		int y = (points[i].y + (height / 2)) / cell_dim;
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
				prev = 0;
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
			if (prev > cells[row][col] + for_jump_diff && row - 2 > 0 && cells[row-2][col] != 0) {
				closest = min(closest, cells[row][col]);
			}
			if (counter > counter_reps) {
				closest = min(closest, min_obj);
			}
			prev = cells[row][col];
		}
		close_vec[col] = closest;
	}
	int left_bound = 0;
	int right_bound = 0;
	float prev = max_dist;
	vector<autocycle_extras::Object> z_boys;

	for (int col = 0; col < cell_col; col++) {
		if (close_vec[col] < max_dist) {
			if (prev == max_dist) {
				left_bound = col;
				right_bound = col;
				prev = close_vec[col];
			} else if (same_obj_diff > abs(prev - close_vec[col])) {
				right_bound++;
				prev = close_vec[col];
			} else {
                if (not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                                     close_vec[left_bound], close_vec[right_bound], obj_lst.obj_lst)) {
                    z_boys.push_back(get_object(left_bound * cell_dim - width / 2,
                                                (right_bound + 1) * cell_dim - width / 2,
                                                close_vec[left_bound],
                                                close_vec[right_bound]));
                }
				prev = max_dist;
			}
		} else if (prev < max_dist) {
            if (not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                                 close_vec[left_bound], close_vec[right_bound], obj_lst.obj_lst)) {
                z_boys.push_back(get_object(left_bound * cell_dim - width / 2,
                                            (right_bound + 1) * cell_dim - width / 2,
                                            close_vec[left_bound],
                                            close_vec[right_bound]));
            }
			prev = max_dist;
		}
	}

    if (prev < max_dist &&
        not intersection(left_bound * cell_dim - width / 2, (right_bound + 1) * cell_dim - width / 2,
                         close_vec[left_bound], close_vec[right_bound], obj_lst.obj_lst)) {
        z_boys.push_back(get_object(left_bound * cell_dim - width / 2,
                                    (right_bound + 1) * cell_dim - width / 2,
                                    close_vec[left_bound],
                                    close_vec[right_bound]));
    }


	cond_objs = condenseObjects(z_boys);
	obj_lst.obj_lst.insert(obj_lst.obj_lst.end(), cond_objs.begin(), cond_objs.end());
        for(auto & i : obj_lst.obj_lst){
	    ROS_INFO_STREAM("OBJECT: (" << i.x1 << ", " << i.x2 << ", " << i.z1 << ", " << i.z2 << ")");
	}

	//auto end = chrono::high_resolution_clock::now();
	//auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
	//ROS_INFO_STREAM("OBJECT DETECTION TOOK: " << (float) duration.count()/1000.0 << " SECONDS");
}

// Gets the latest roll
void get_roll(const std_msgs::Float32 data){
    roll = data.data;
}

// Gets the latest Heading
void get_heading(const std_msgs::Float32 data){
    heading = data.data;
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
                        x = *((uint32_t *) buff);

                        // y val. We can record it if we determine we need it
                        file.read(buff, 4);
                        y = *((uint32_t *) buff);

                        // z val
                        file.read(buff, 4);
                        z = *((uint32_t *) buff);

                        //xs.push_back(x);
                        //zs.push_back(z);

                        // Ignores tag and reflexivity
                        file.ignore(2);

                        autocycle_extras::Point p;
                        p.z = x;
                        p.x = y;
                        p.y = z;
			if(p.z !=0 && p.x > -half_width && p.x < half_width && p.y > -half_height && p.y < half_height){
			    points.push_back(p);
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
    points.shrink_to_fit();
    //o_file << endl;
}

// Adjusts detected points for roll from the bike
void fix_roll(){
  // Initializes variables
  autocycle_extras::Point p;
  autocycle_extras::Point np;

  // Updates each point in `req` and pushes it to the new vector
  for(int i=0;i<points.size();i++){
    p = points[i];
    np.x = (p.x*cos(roll)) - (p.y*sin(roll));
    np.y = (p.x*sin(roll)) + (p.y*cos(roll));
    np.z = p.z;
    points[i] = np;
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

  // Creates subscriber that updates velocity
  ros::Subscriber vel_sub = nh.subscribe("sensors/vel", 1, &get_velocity);

  // Creates subscriber that updates velocity
  ros::Subscriber ready_sub = nh.subscribe("cycle/frame_ready", 1, &update_ready);

  // Creates server proxy for calculating new deltas
  calc_deltas = nh.advertise<autocycle_extras::CalcDeltas>("cycle/path", 1);

  // Sets desired heading (for now the initial heading)
  while(heading == -1){
      ros::spinOnce();
  }

  // Sets initial desired heading
  des_heading = heading;

  // Reserves the requisite space for the vectors in use
  points.reserve(15000);
  path.reserve(400);
  xs.reserve(400);
  ys.reserve(400);

  // Starts the clock
  auto start = chrono::high_resolution_clock::now();

  // starts second clock used only for updating object positions
  state_start = std::chrono::high_resolution_clock::now();


  // Tells ROS to look for any callbacks waiting to run
  ros::spinOnce();

  // Clears all points in preparation for new Livox data
  points.clear();

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
  generate_curve();

  // Clears the lvx file and shuts down. Mission Complete
  f_done.open(path_to_lvx, ios::trunc);
  f_done.write("done", 4);
  f_done.close();
  //o_file.close();
  return 0;
}
