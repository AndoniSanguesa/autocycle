#include <cmath>
#include <set>
#include <vector>
#include <functional>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <autocycle_extras::Curve.h>
#include <autocycle_extras::CalcDeltas.h>

// Size of plot
int width = 20;
int height = 20;

// Size of each node
int node_size = 1;

// The dimensions of the graph (how many nodes in each direction)
int x_dim = width / node_size;
int y_dim = height / node_size;

// The nodes that have been deemed blocked by an object
std::unordered_set<int> blocked_nodes;
std::unordered_set<int> center_blocked_nodes;

// The path being taken
std::vector<std::tuple<float, float>> path;

// Starting node
std::tuple<int, int> start_node;

// End node
std::tuple<int, int> end_node;

// x values and y values
std::vector<int> xs;
std::vector<int> ys;

// Heading
int desired_heading = 0;

// Calculate Padding
float padding = 1.5;
int padding_num = (int) (padding / (float) node_size);

// Theta to adjust heading by
float theta = 0;
float des_heading = 0;
float heading = -1;

// ROS stuff
ros::ServiceClient new_data;
ros::ServiceClient calc_deltas;
std_srvs::CalcDeltas::Request calc_deltas_req;
std_srvs::CalcDeltas::Response calc_deltas_resp;
ros::ServiceCleint new_path_cli;
std_srvs::Empty::Request empty_req;
std_srvs::Empty::Response empty_resp;

int cantor(std::tuple<int, int> node){
    // Maps a tuple to 2 integers to a unique integer (for hashing)
    int p1 = std::get<0>(node);
    int p2 = std::get<1>(node);
    return (((p1 + p2) * (p1 + p2 +1))/2) + p2;
}

std::tuple<float, float> get_change(std::tuple<float, float> p1, std::tuple<float, float> p2){
    // Calculates difference between nodes
    return(std::make_tuple(std::get<0>(p2) - std::get<0>(p1), std::get<1>(p2) - std::get<1>(p1)));
}

void augment_path(){
    // Generates intermediary points to assist in interpolation
    std::vector<std::tuple<float, float>> new_path;
    std::tuple<float, float> change;
    float new_x, new_y = 0;
    unsigned long last_ind;

    new_path.reserve(path.size()*2);
    new_path.push_back(path[0]);
    xs.clear();
    ys.clear();

    for(int i = 1; i < path.size(); i++){
        last_ind = new_path.size()-1;
        change = get_change(new_path[last_ind], path[i]);
        new_x = std::get<1>(new_path[last_ind])+(std::get<0>(change)*0.5);
        new_y = std::get<0>(new_path[last_ind])+(std::get<1>(change)*0.5);
        new_path.emplace_back(new_y, new_x);
        new_path.push_back(path[i]);
        xs.push_back(new_x);
        ys.push_back(new_y);
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

std::tuple<int, int> get_node_from_point(std::tuple<float, float> point){
    //Takes cartesian point and returns the corresponding node
    float x = std::get<0>(point);
    float y = std::get<1>(point);
    return (std::make_tuple(
            (int) (((-y) + ((float) height / 2)) / (float) node_size),
            (int) (x / (float) node_size)
    ));
}

void get_blocked_nodes(std::tuple<float, float, float, float> obj){
    // Returns the list of nodes that an object is blocking
    float x1, x2, y1, y2, tmp, m;
    double delta_x;
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

    double cur_point [2] = {x1, y1};
    bool vert = x2 - x1 == 0;
    if(!vert){
        m = (y2 - y1) / (x2 - x1);
        delta_x = node_size/sqrt(1+pow(m, 2));
    }

    while((!vert && (cur_point[0] < x2)) || (vert && (cur_point[1] < y2))){
        node = get_node_from_point(std::make_tuple(cur_point[0], cur_point[1]));
        if(center_blocked_nodes.find(cantor(node)) == center_blocked_nodes.end()){
            center_blocked_nodes.insert(cantor(node));
            std::vector<std::tuple<int, int>> cur_blocked;
            x = std::get<1>(node);
            y = std::get<0>(node);

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

    node = get_node_from_point(std::make_tuple(x2, y2));
    if(center_blocked_nodes.find(cantor(node)) == center_blocked_nodes.end()){
        center_blocked_nodes.insert(cantor(node));
        std::vector<std::tuple<int, int>> cur_blocked;
        x = std::get<1>(node);
        y = std::get<0>(node);

        for(int y_ind = -1; y_ind < 2; y_ind++){
            for(int x_ind = -1; x_ind < 2; x_ind++){
                if(0 <= y+y_ind < y_dim && 0 <= x+x_ind < x_dim){
                    cur_blocked.emplace_back(y+y_ind, x+x_ind);
                }
            }
        }
    }
}

void bfs(){
    // Finds the shortest path from the starting node to the
    // end node via a Breadth First Search(BFS)
    std::deque<std::tuple<int, int>> q;
    std::unordered_set<int> visited;
    std::unordered_map<int, std::tuple<int, int>> parent;
    std::tuple<int, int> next_node, node;
    int x, y;
    int tot_count = 0; // DELETE THIS
    q.push_back(start_node);

    while(!q.empty()){
        next_node = q.front();
        q.pop_front();

        y = std::get<0>(next_node);
        x = std::get<1>(next_node);

        std::tuple<int, int> adj_nodes [5] = {std::make_tuple(y, x+1), std::make_tuple(y-1, x+1), std::make_tuple(y+1, x+1), std::make_tuple(y+1, x), std::make_tuple(y-1, x)};
        visited.insert(cantor(next_node));

        for(auto & adj_node : adj_nodes){
            node = adj_node;

            if(!(0 <= std::get<0>(node) && std::get<0>(node) <= y_dim) || !(0 <= std::get<1>(node) && std::get<1>(node) <= x_dim) || blocked_nodes.find(cantor(node)) != blocked_nodes.end()){
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
        xs.push_back(std::get<1>(node));
        ys.push_back(std::get<0>(node));
        try {
            node = parent.at(cantor(node));
        } catch (std::out_of_range const &) {
            //ROS_INFO_STREAM("THE BIKE SHOULD BE STOPPED"); // TODO: stop the bike
            break;
        }
    }
    path.emplace_back(start_node);
    std::reverse(path.begin(),path.end());
}

void update_end_node() {
    // Updates end_node to approximate the desired heading

    double m = tan(theta);
    int half_height = height / 2;
    std::tuple<int, int> top = get_node_from_point(std::make_tuple(half_height / m, half_height));
    std::tuple<int, int> bot = get_node_from_point(std::make_tuple((-half_height) / m, -half_height));
    std::tuple<int, int> right = get_node_from_point(std::make_tuple(width, m * (width)));

    if (0 <= std::get<1>(top) && std::get<1>(top) < x_dim) {
        end_node = top;
    } else if (0 <= std::get<1>(bot) && std::get<1>(bot) < x_dim) {
        end_node = bot;
    } else {
        end_node = right;
    }
}

void generate_curve(const std::vector<std::tuple<float, float, float, float>>& obj_lst) {
    // Creates the curve around objects
    ros::spinOnce();
    theta = des_heading - heading;

    reset_vars();
    update_end_node();
    std::vector<std::tuple<float, float, float, float>> real_obj_lst;
    real_obj_lst.reserve(obj_lst.size());

    for(auto & i : obj_lst){
        real_obj_lst.emplace_back(std::make_tuple(std::get<2>(i)/1000, std::get<3>(i)/1000, std::get<0>(i)/1000, std::get<1>(i)/1000));
    }

    for (auto & i : real_obj_lst) {
        get_blocked_nodes(i);
    }
    bfs();

    calc_deltas_req.path_x = xs;
    calc_deltas_req.path_y = ys;
    calc_deltas(calc_deltas_req, calc_deltas_resp);
    new_path_cli(empty_req, empty_resp);
    new_data(empty_req, empty_resp);
}

void get_heading(const std_msgs::Float32 data){
    heading = data.data;
}

int main(int argc, char **argv){
    // Starts the ros node and prepares the relevant services

    // Reserves vector space when possible
    path.reserve(400);
    xs.reserve(400);
    ys.reserve(400);

    // Initialize the node and register it with the master.
    ros::init(argc, argv, "bezier");
    ros::NodeHandle nh;

    // Waits for data getter service
    ros::waitForService("collect_data");

    // Waits for data getter service
    ros::waitForService("calculate_deltas");

    // Creates server proxy for collecting another set of data
    new_data = nh.serviceClient<std_srvs::Empty>("collect_data");

    // Creates server proxy for calculating new deltas
    calc_deltas = nh.serviceClient<autocycle_extras::CalcDeltas>("calculate_deltas");

    // Creates Publisher that alerts send_action that it needs to reset its coordinate system
    new_path_cli = nh.serviceClient<std_srvs::Empty>("reset_action");

    // Creates subscriber that updates the heading
    ros::Subscriber head_sub = nh.subscribe("sensors/heading", 1, &get_heading);

    // Sets desired heading (for now the initial heading)
    while(heading == -1){
        ros::spinOnce();
    }

    // Sets initial desired heading
    des_heading = heading;

    // Creates subscriber that will generate a path when there is new data
    ros::Subscriber = nh.subscribe("cycle/object_frame", 1, &generate_curve);

    // Asks for initial data
    new_data(new_data_req, new_data_resp);

    // Spins
    ros::spin();
}