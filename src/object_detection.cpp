#include <math.h> 
#include <vector>
#include <tuple>
#include <autocycle/Object.h>
#include <autocycle_extras/Point.h>

using namespace std;

// Global variable declarations
int height = 10000;   // vertical dimension in millimeters
int width = 20000;    // horizontal dimension in millimeters
int cell_dim = 50;    // dimension of cells in millimeters (cells are squares)

int cell_row = ceil((1.0 * height) / cell_dim);
int cell_col = ceil((1.0 * width) / cell_dim);

// Tunable parameters to determine if something is an object.
int col_diff = 50;                       // Expected max difference between two adjacent cells in a column.
int for_jump_diff = col_diff * 1.5;      // Expected min difference between cells in a column to indicate a jump forward.
int counter_reps = 2;                    // Number of reps required to dictate it is an object.
int same_obj_diff = 150;                 // maximum diff between horizontal cells to be considered the same object
int group_dist = 1500;					 // max dist between adjacent objects for convex hull
float max_dist = 200000;				 // max z distance for objects


class Obj {
	public:
		int x1, x2, z1, z2;
		Obj(int, int, int, int);
};

Obj::Obj(int a, int b, int c, int d) {
	x1 = a;
  	x2 = b;
  	z1 = c;
  	z2 = d;
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
	vector<vector<int>> adj(a);
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

float pointToSeg(int x, int z, Obj seg) {
	float segx = seg.x1 - seg.x2;
	float segz = seg.z1 - seg.z2;
	float lpx = seg.x1 - x;
	float lpz = seg.z1 - z;
	float seg_len = sqrt(pow(seg.x1 - seg.x2, 2) + pow(seg.z1 - seg.z2, 2));
	if (seg_len == 0) {
		return sqrt(pow(seg.x1 - x, 2) + pow(seg.z1 - z, 2));
	}
	float t = segx / seg_len * x / seg_len + segz / seg_len * z / seg_len;
	if (t < 0) {
		t = 0;
	} else if (t > 0) {
		t = 1;
	}
	float nx = segx * t;
	float nz = segz * t;
	return sqrt(pow(nx - x, 2) + pow(nz - z, 2));
}

float segDist(Obj seg1, Obj seg2) {
	float dist = pointToSeg(seg1.x1, seg1.z1, seg2);
	float temp = pointToSeg(seg1.x2, seg1.z2, seg2);
	if (dist > temp) {
		dist = temp;
	}
	float temp = pointToSeg(seg2.x1, seg2.z1, seg1);
	if (dist > temp) {
		dist = temp;
	}
	float temp = pointToSeg(seg2.x2, seg2.z2, seg1);
	if (dist > temp) {
		dist = temp;
	}
	return dist;
}

int leftMost(vector<int[2]> points) {
	int min_ind;
	for (int i; i < points.size(); i++) {
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

int orientation(int p[2],int q[2],int r[2]){
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

vector<Obj> convHull(vector<Obj> objects) {
	vector<int[2]> points;
	for (int i; i < objects.size(); i++) {
		points.push_back({objects[i].x1, objects[i].z1});
		points.push_back({objects[i].x2, objects[i].z2});
	}
	int n = points.size();
	if (n < 3){
		return objects;
	}
	int l = leftMost(points);
	vector<int> hull;
	int p = l;
	int q;

	while(true) {
		hull.push_back(p);
		q = (p+1) % n;
		for (int i; i < n; i++) {
			if (orientation(points[p], points[i], points[q]) == 2) {
				q = i;
			}
		}
		p = q;
		if (p == l) {
			break;
		}
	}
	vector<Obj> objects;
	int prev;
	for (int i = 1; i < hull.size(); i++) {
		objects.push_back(Obj(points[hull[prev]][0], points[hull[i]][0], points[hull[prev]][1], points[hull[i]][1]));
	}
	return objects;
}

vector<Obj> condenseObjects(vector<Obj> objects) {
	Graph gr = Graph(objects.size());
	for (int x; x < objects.size(); x++) {
		for (int y = x+1; y < objects.size(); y++) {
			if (segDist(objects[x], objects[y]) < group_dist) {
				gr.addEdge(x,y);
			}
		}
	}
	vector<vector<int>> groups = gr.connectedComps();
	vector<vector<Obj>> grouped_objs;
	for (int i; i < groups.size(); i++) {
		vector<Obj> temp;
		for (int y; y < groups[i].size(); y++) {
			temp.push_back(objects[y]);
		}
		grouped_objs.push_back(temp);
	}
	vector<Obj> new_objects;
	for (int i; i < grouped_objs.size(); i++) {
		vector<Obj> temp = convHull(grouped_objs[i]);
		new_objects.insert(new_objects.end(), temp.begin(), temp.end());
	}
	return new_objects;
}

void object_detection(vector<autocycle_extras::Point> points) {
	vector<vectors<float>> cells(cell_row, vector<int>(cell_col, max_dist));
	for (int i; i < points.size(); i++) {
		float z = points[i].z;
		int x = (points[i].x + (width / 2)) / cell_dim;
		int y = (points[i].y + (height / 2)) / cell_dim;
		if (x >= 0 &&  x < cell_col && y >=0 && y < cell_row && z < cells[y][x]) {
			cells[y][x] = z;
		}
	}
	vector<float> close_vec(cell_col);
	for (int col; col < cell_col; col++) {
		float prev;
		float closest = max_dist;
		int counter;
		float min_obj = max_dist;

		for (int row: row < cell_row; row++) {
			if (cells[row][col] == 0) {
				counter = 0;
				min_obj = max_dist;
				prev = 0;
				continue;
			}
			if (abs(cells[row][col] - prev) > col_diff) {
				counter = 0;
				min_obj = 0;
			} else {
				counter += 1;
				min_obj = min(min_obj, cells[row][col]);
			}
			if (prev > cells[row][col] + for_jump_diff && row - 2 > 0 && cells[row-2][col] != 0) {
				closest = min(closest, cells[row][col]);
			}
			if (counter > counter_reps) {
				closest = min(closest, min_obj);
			}
			prev = cells[row][col];
		}
		close_arr[col] = closest

	}
	int left_bound = 0;
	int right_bound = 0;
	float prev = max_dist;
	vector<Obj> z_boys;

	for (int col; col < cell_col; col++) {
		if (close_arr[col] < max_dist) {
			if (prev == max_dist) {
				left_bound = col;
				right_bound = col;
				prev = close_arr[col];
			} else if (same_obj_diff > abs(prev - close_arr[col])) {
				right_bound++;
				prev = close_arr[col]
			} else {
				z_boys.push_back(Obj(left_bound * cell_dim - width / 2,
									(right_bound + 1) * cell_dim - width / 2,
                             		close_arr[0, left_bound],
									close_arr[0, right_bound]));
				prev = max_dist;
			}
		} else if (prev < max_dist) {
			z_boys.push_back(Obj(left_bound * cell_dim - width / 2,
									(right_bound + 1) * cell_dim - width / 2,
                             		close_arr[0, left_bound],
									close_arr[0, right_bound]));
			prev = max_dist;
		}
	}

	if (prev < max_dist) {
		z_boys.push_back(Obj(left_bound * cell_dim - width / 2,
									(right_bound + 1) * cell_dim - width / 2,
                             		close_arr[0, left_bound],
									close_arr[0, right_bound]));
	}

	vector<Obj> z_boys = condenseObjects(z_boys);
	vector<autocycle::Object>to_pub;
	for (int i; i < z_boys.size(); i++) {
		to_pub.push_back(autocycle::Object(z_boys.x1, z_boys.x2, z_boys.z1, z_boys.z2));
	}

}






//     print("THIS SHOULD BE PUBLISHING")
//     for thing in z_boys:
//         to_pub.append(Object(thing.x1, thing.x2, thing.z1, thing.z2))
//     bruh = pub.publish(to_pub, iden2)
//     iden2 += 1
//     print(bruh)
//     for o in to_pub:
//         print(f"({o.x1}, {o.x2}, {o.z1}, {o.z2})")
//     return []