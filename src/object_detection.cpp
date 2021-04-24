#include <math.h> 
#include <vector>
#include <tuple>
#include <ros/ros.h>
#include <autocycle_extras/Object.h>
#include <autocycle_extras/Point.h>
#include <autocycle_extras/DetectObjects.h>
#include <autocycle_extras/ObjectList.h>
#include <chrono>
using namespace std;

// Global variable declarations
int height = 3000;   // vertical dimension in millimeters
int width = 20000;    // horizontal dimension in millimeters
int cell_dim = 50;    // dimension of cells in millimeters (cells are squares)

int cell_row = ceil((1.0 * height) / cell_dim);
int cell_col = ceil((1.0 * width) / cell_dim);
int old_tracking_id = -1;
int tracking_id = 0;

autocycle_extras::ObjectList pub_lst;
autocycle_extras::ObjectList tracking_ol;
ros::Publisher obj_pub;

// Tunable parameters to determine if something is an object.
int col_diff = 50;                       // Expected max difference between two adjacent cells in a column.
int for_jump_diff = col_diff * 1.5;      // Expected min difference between cells in a column to indicate a jump forward.
int counter_reps = 2;                    // Number of reps required to dictate it is an object.
int same_obj_diff = 150;                 // maximum diff between horizontal cells to be considered the same object
int group_dist = 1500;					 // max dist between adjacent objects for convex hull
float max_dist = 200000;				 // max z distance for objects


autocycle_extras::Object get_object(float x1, float x2, float z1, float z2){
	autocycle_extras::Object obj;
	obj.x1 = x1;
	obj.x2 = x2;
	obj.z1 = z1;
	obj.z2 = z2;
	return obj;
}

class Graph {
		int V;
		vector <vector<int>> adj;
	public:
		Graph(int);
		void addEdge(int, int);
		void dfs(vector<int> &temp, int vert, vector<bool> &visited);
		vector<vector<int>> connectedComps();
};

Graph::Graph(int a) {
	V = a;
	adj.resize(a);
}

void Graph::addEdge(int a, int b) {
	adj[a].push_back(b);
	adj[b].push_back(a);
}

void Graph::dfs(vector<int> &temp, int vert, vector<bool> &visited) {
	visited[vert] = true;
	temp.push_back(vert);
	for (int i = 0; i < adj[vert].size(); i++) {
		if (!visited[adj[vert][i]]) {
			dfs(temp, adj[vert][i], visited);
		}
	}
}

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


bool object_detection(
    autocycle_extras::DetectObjects::Request &req,
    autocycle_extras::DetectObjects::Response &resp
) {
    auto start = std::chrono::high_resolution_clock::now();
    while (old_tracking_id != -1 && old_tracking_id == tracking_id){
        ros::spinOnce();
    }
    old_tracking_id = tracking_id;
    vector<autocycle_extras::Point> points = req.data;
	vector<vector<float>> cells(cell_row, vector<float>(cell_col, max_dist));
	for (int i = 0; i < points.size(); i++) {
		float z = points[i].z;
		int x = (points[i].x + (width / 2)) / cell_dim;
		int y = (points[i].y + (height / 2)) / cell_dim;
		if (z < cells[y][x]) {
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
				z_boys.push_back(get_object(left_bound * cell_dim - width / 2,
						  (right_bound + 1) * cell_dim - width / 2,
                             		          close_vec[left_bound],
						  close_vec[right_bound]));
				prev = max_dist;
			}
		} else if (prev < max_dist) {
			z_boys.push_back(get_object(left_bound * cell_dim - width / 2,
						  (right_bound + 1) * cell_dim - width / 2,
                             		          close_vec[left_bound],
						  close_vec[right_bound]));
			prev = max_dist;
		}
	}

	if (prev < max_dist) {
		z_boys.push_back(get_object(left_bound * cell_dim - width / 2,
						  (right_bound + 1) * cell_dim - width / 2,
                             		      	  close_vec[left_bound],
			  		          close_vec[right_bound]));
	}

	pub_lst.obj_lst = condenseObjects(z_boys);
        pub_lst.iden = tracking_id;
	obj_pub.publish(pub_lst);
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	ROS_INFO_STREAM("OBJECT DETECTION TOOK: " << (float) duration.count()/1000.0 << " SECONDS");
    return true;
}

void get_new_frame(const autocycle_extras::ObjectList new_ol){
    tracking_ol = new_ol;
    tracking_id++;
}

int main(int argc, char **argv){
    // Registers node with the master
    ros::init(argc, argv, "object_dection");
    ros::NodeHandle nh;
    // Creates Service to be called
    ros::ServiceServer obj_det = nh.advertiseService("object_detection", &object_detection);

    // Creates Subscriber that subscribes to the tracking frames
    ros::Subscriber track_sub = nh.subscribe("cycle/object_frame", 1, &get_new_frame);

    // Creates Object List Publisher
    obj_pub = nh.advertise<autocycle_extras::ObjectList>("cycle/objects", 1);

    // Waits to be called
    ros::spin();
}
