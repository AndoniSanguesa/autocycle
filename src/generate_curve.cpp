#include <ros/ros.h>
#include <math.h>

// Size of plot
int width = 20;
int height = 20;

// Size of each node
int node_size = 2;

// The dimensions of the graph (how many nodes in each direction)
int x_dim = width / node_size;
int y_dim = height / node_size;

// The nodes that have been deemed blocked by an object
std::set<std::tuple<int, int>> blocked_nodes;
std::set<std::tuple<int, int>> center_blocked_nodes;

// The path being taken
std::vector<std::tuple<float, float>> path;
path.reserve(400);

// Starting node
int start_node = std::make_tuple(y_dim / 2, 0);

// End node
int end_node;

// Heading
int desired_heading = 0;

// Calculate Padding
float padding = 1.6;
int padding_num = (int) (padding / (float) node_size);

std::tuple<float, float> get_change(p1, p2){
    // Calculates difference between nodes
    return(std::make_tuple(p2.get(0) - p1.get(0), p2.get(1) - p1.get(1)));
}

void augment_path(path){
    // Generates intermediary points to assist in interpolation
    std::vector<std::tuple<float, float>> new_path;
    new_path.reserve(800);
    new_path.push_back(path[0]);

    for(int i = 1; i < path.size(); i++){
        change = get_change(new_path[new_path.size()-1], path[i]);
        new_path.push_back(std::make_tuple(new_path[-1].get(0)+(change.get(0)*0.5), new_path[-1].get(1)+(change.get(1)*0.5)));
        new_path.push_back(path[ind]);
    }

    path = new_path;
}

void reset_vars(){
    // Resets variables between calls

    // The nodes that have been deemed blocked by an object
    blocked_nodes.clear();
    center_blocked_nodes.clear();

    // The path being taken
    path.clear();
    path.reserve(400);
}

std::tuple<int, int> get_node_from_point(point){
    //Takes cartesian point and returns the corresponding node
    int x = point.get(0);
    int y = point.get(1);
    return (std::make_tuple(
        ((-y) + (height / 2)) / node_size,
        x / node_size
    ));
}

void get_blocked_nodes(obj){
    // Returns the list of nodes that an object is blocking
    float x1, x2, y1, y2, tmp, m, delta_x;
    int x, y;
    std::tuple<int, int> node, new_node;
    std::tie (x1, x2, y1, y2) = obj;

    if(x2 < x1){
        tmp = x1;
        x1 = x2;
        x2 = tmp;

        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    float cur_point [2] = {x1, y1};
    bool vert = x2 - x1 == 0;
    if(!vert){
        m = (y2 - y1) / (x2 - x1);
        delta_x = node_size/sqrt(1+pow(m, 2));
    }

    while((!vert && (cur_point[0] < x2)) || (vert && (cur_point[1] < y2))){
        node = get_node_from_point(cur_point);
        if(center_blocked_nodes.find(node) == center_blocked_nodes.end()){
            center_blocked_nodes.insert(node);
            std::vector<std::tuple<int, int>> cur_blocked;
            x = node.get(1);
            y = node.get(0);

            for(int y_ind = -1; y_ind < 2; y_ind++){
                for(int x_ind = -1); x_ind < 2; x_ind++){
                    if(0 <= y+y_ind < y_dim && 0 <= x+x_ind < x_dim){
                        cur_blocked.push_back(std::make_tuple(y+y_ind, x+x_ind));
                    }
                }
            }

            for(int ind; ind < cur_blocked.size(); ind++){
                blocked_nodes.insert(cur_blocked[ind]);
            }
        }
        if(!vert){
            cur_point[0] = cur_point[0] + delta_x
            cur_point[1] = cur_point[1] + (m * delta_x)
        } else:
            cur_point[1] = cur_point[1] + node_size
    }

    node = get_node_from_point(std::make_tuple(x2, y2));
    if(center_blocked_nodes.find(node) == center_blocked_nodes.end()){
        center_blocked_nodes.insert(node);
        std::vector<std::tuple<int, int>> cur_blocked;
        x = node.get(1);
        y = node.get(0);

        for(int y_ind = -1; y_ind < 2; y_ind++){
            for(int x_ind = -1); x_ind < 2; x_ind++){
                if(0 <= y+y_ind < y_dim && 0 <= x+x_ind < x_dim){
                    cur_blocked.push_back(std::make_tuple(y+y_ind, x+x_ind));
                }
            }
        }
    }
}

void bfs(){
    // Finds the shortest path from the starting node to the
    // end node via a Breadth First Search(BFS)
    std::deque<std::tuple<int, int>> q;
    std::set<std::tuple<int, int>> visited;
    std::unordered_map<tuple<int, int>,tuple<int, int> parent;
    std::tuple<int, int> next_node, node;
    ;
    int x, y;

    q.push_back(start_node);

    while(!queue.empty()){
        next_node = q.pop();
        y = next_node.get(0);
        x = next_node.get(1);
        std::tuple<int, int> adj_nodes [5] = {std::make_tuple(y, x+1), std::make_tuple(y-1, x+1), std::make_tuple(y+1, x+1), std::make_tuple(y+1, x), std::make_tuple(y-1, x);
        visited.insert(next_node);
        for(ind i; i < 5; i++){
            node = adj_nodes[i];
            if(!(0 <= node.get(0) && node.get(0) <= y_dim) || !(0 <= node.get(1) && node.get(1) <= x_dim) || blocked_nodes.find(node) != blocked_nodes.end()){
                continue;
            }
            if(visited.find(node) != visited.end() || q.find(node) != q.end()){
                continue
            }
            q.push_back(node);
            parent[node] = next_node;
            if(node == end_node){
                queue.clear();
                break;
            }
        }
    }

    node = end_node;
    while(node != start_node){
        path.push_back(cur_node);
        try{
            node = parent.at(cur_node)
        } catch (const std::out_of_range){}
            ROS_INFO_STREAM("THE BIKE SHOULD BE STOPPED"); // TODO: stop the bike
            break;
        }
    }
    path.push_back(start_node);
}