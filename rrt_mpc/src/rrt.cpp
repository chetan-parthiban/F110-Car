#include "rrt/rrt.h"
#include <chrono>

// Destructor of the RRT class
RRT::~RRT() {
    ROS_INFO_STREAM("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh, Predictor &pred): nh_(nh), predictor(pred), gen((std::random_device())()) { 

    // Params for subs and pubs
    std::string pose_topic, scan_topic, map_topic, occgrid_topic, tree_topic, test_topic, line_topic,best_line_topic, odom_topic;
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("map_topic", map_topic);
    nh_.getParam("viz_topic",occgrid_topic);
    nh_.getParam("min_scan_dist", min_scan_dist);
    nh_.getParam("test_topic",test_topic);
    nh_.getParam("tree_topic", tree_topic);
    nh_.getParam("line_topic", line_topic);
    nh_.getParam("best_line_topic",best_line_topic);
    nh_.getParam("do_visualization",do_visualization);
    nh_.getParam("odom_topic", odom_topic);
    
    // Params for occupancy grids 
    nh_.getParam("lidar_offset",lidar_offset);
    nh_.getParam("x_sample",x_sample);
    nh_.getParam("y_sample",y_sample);
    
    // Params for RRT
    nh_.getParam("steer_dist",steer_dist);
    nh_.getParam("RRTs_connection_dist",RRTs_connection_dist);
    nh_.getParam("tree_nodes",tree_nodes);
    nh_.getParam("car_intersection_radius",car_intersection_radius);
    nh_.getParam("max_iter",max_iter);
    nh_.getParam("rrt_close_to_goal",rrt_close_to_goal_dist);
    
    //PREDICTION ADDITION
    nh_.getParam("nominal_speed",nominal_speed);
    nh_.getParam("dynamic_collision_distance",dynamic_collision_distance);
    //END PREDICTION ADDITION

    // Initialize random generators
    x_dist = std::uniform_real_distribution<>(lidar_offset,x_sample);
    y_dist = std::uniform_real_distribution<>(-y_sample,y_sample);
    gen = std::mt19937(ros::Time().nsec);
    
    //LOAD WAYPOINTS
    initialize_points();

    // Create a occupancy grid
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    nav_msgs::OccupancyGrid map_msg;
    ROS_INFO_STREAM("map topic: " + map_topic);
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic);
    if (map_ptr != NULL)
    {
        map_msg = *map_ptr;
    }
    ROS_INFO_STREAM("GOT MAP MESSAGE");
    occ_grid_ = OccGrid(map_msg);
    ROS_INFO_STREAM("BUILT OCCGRID");

    // Create subs and pubs
    // scan_sub_ = nh_.subscribe(scan_topic, 1, &RRT::scan_callback, this);
    odom_sub_  = nh_.subscribe(odom_topic, 1, &RRT::get_target, this);
    occgrid_pub = nh_.advertise<visualization_msgs::Marker>(occgrid_topic,1);
    tree_pub = nh_.advertise<visualization_msgs::Marker>(tree_topic,1);
    line_pub = nh_.advertise<visualization_msgs::Marker>(line_topic,1);
    best_line_pub = nh_.advertise<visualization_msgs::Marker>(best_line_topic,1);

    ROS_INFO_STREAM("Finished connecting to topics");
}

//**************************CALLBACKS*****************************
//****************************************************************
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // ROS_INFO_STREAM("rrt scan_callback");

    if(!pose_init) {
        return;
    }
    else{
        //occ_grid_.clear_dynamic_grid();
        // for (size_t i = 0; i < scan_msg->ranges.size(); i++)
        // {
        //     double r = scan_msg->ranges[i];
        //     if (r >= scan_msg->range_min && r <= scan_msg->range_max)
        //     {
        //         double theta = scan_msg->angle_min + i*scan_msg->angle_increment;
        //         Eigen::Vector2d scan_xy_car_frame;
        //         scan_xy_car_frame << r*std::cos(theta), r*std::sin(theta);
        //         Eigen::Vector2d scan_xy_map_frame = transform_scan_to_map_frame(most_recent_pose_, scan_xy_car_frame);
        //         occ_grid_.set_dynamic_grid_point(scan_xy_map_frame);
        //     }
        // }        
        // if (do_visualization) occgrid_pub.publish(occ_grid_.get_vis_occ_grid_msg()); 
    }    
}
void RRT::get_target(const nav_msgs::Odometry::ConstPtr &pose_msg) {
    // takes in the current odometry and outputs a goal waypoint in the car frame
    // Update the car's current pose
    // ROS_INFO_STREAM("rrt get_target callback");

    geometry_msgs::PoseStamped geo_pose;
    geo_pose.pose = pose_msg->pose.pose;
    geo_pose.pose.orientation = pose_msg->pose.pose.orientation;
    most_recent_pose_ = geo_pose;
    pose_init = true;

    // Compute and return goal waypoint
    std::vector<Eigen::Vector2d> interscetions = find_intersections(most_recent_pose_,car_intersection_radius);
    if(interscetions.size() == 0){
        ROS_INFO_STREAM("NO INTERSECTIONS");
        Eigen::Vector2d default_vec = target; // local_path[local_path.size()-2].point;
        target = default_vec;
        return;
    }
    else{
        current_goal = find_best_valid_point(interscetions);  
        // ROS_INFO_STREAM("starting_rrt");
        run_RRT();
    }

    // visualize waypoints if we want to
    if (do_visualization) visualize_waypoints();
}

//****************************RRT Top Level******************************
//***********************************************************************
Eigen::Vector2d RRT::run_RRT(){
    
    // setup a base case vector 
    Eigen::Vector2d base = target;
    
    // update predictions
    if(predictor.prediction_ready) {
         std::vector<Eigen::VectorXd> pred = predictor.get_path();
        current_prediction_dt = predictor.get_dt();
        current_prediction.clear();
        // ROS_INFO_STREAM("translating to v2d, size: ");
        // ROS_INFO_STREAM(pred.size());
        for(int i = 0; i < pred.size(); i++){
            current_prediction.push_back(Eigen::Vector2d(pred[i](0),pred[i](1)));
        }
    }

    // iniitalize the tree
    std::vector<Node> tree;
    Node root;
    root.point = Eigen::Vector2d::Zero();
    root.point[0] = lidar_offset;
    root.is_root = true;
    root.cost = 0;
    root.parent = -1;
    add_node(tree,root);

    // ROS_INFO_STREAM("starting to build tree");
    // grow tree to N nodes (abstract to parameter for tuning)
    int c = 0;
    while(tree.size() < tree_nodes && c < max_iter){
        // ROS_INFO_STREAM(c);
        Eigen::Vector2d pt = sample();
        int par = nearest(tree, pt);
        // ROS_INFO_STREAM("steer");
        Node n = steer(tree[par],par,pt);
        // ROS_INFO_STREAM("collision");
        if(!check_collision(tree[par],n)){
            // ROS_INFO_STREAM("adding node");
           add_node(tree,n);  
        }
        c++;
    }
    // ROS_INFO_STREAM("adding old path");
    // update tree with previous path
    if(previous_path_init){
        for(Node p_node: recover_best_path()){
            int par = nearest(tree, p_node.point);
            Node n = steer(tree[par],par,p_node.point);
            if(!check_collision(tree[par],n)){
                add_node(tree,n);

            }
        }
    }        

    //find best path given current goal and tree
    local_path = find_path(tree,current_goal);

    // ROS_INFO_STREAM("Smoothing path");
    // save and smooth current path
    save_best_path(local_path);
    local_path = smooth_path(local_path);

    // visualizations
    if (do_visualization){
        visualize_tree(tree);
        visualize_best_path(local_path);
    }
        // ROS_INFO_STREAM("visuals done");


    // Select and return waypoint
    if(local_path.size() < 2) {
        // ROS_INFO_STREAM("PATH TOO SMALL");
        return base;
    }
    target = local_path[local_path.size()-2].point;
    target = car_frame_to_map_frame(target);
    return target; // car_frame_to_map_frame(target);
}

void RRT::RRTstar(std::vector<Node> &tree, int ind){
    std::vector<int> neighbors = find_neighbors(tree,ind,RRTs_connection_dist);
    
    //switch parent of the current node
    for(int i : neighbors){
        if(tree[ind].cost > tree[i].cost + line_cost(tree[ind],tree[i])){
            if(check_collision(tree[i],tree[ind])) continue;
            tree[tree[ind].parent].children.erase(ind);
            tree[ind].parent = i;
            tree[tree[ind].parent].children.insert(ind);
            tree[ind].cost = tree[i].cost + line_cost(tree[ind],tree[i]);
            update_cost_rec(tree,ind);
        }
    }

    //switch the parent of the neighboring nodes
    for(int i:neighbors){
        if(tree[i].cost > tree[ind].cost + line_cost(tree[ind],tree[i])){
            if(check_collision(tree[i],tree[ind])) continue;
            tree[tree[i].parent].children.erase(i);
            tree[i].parent = ind;
            tree[tree[i].parent].children.insert(i);
            tree[i].cost = tree[ind].cost + line_cost(tree[ind],tree[i]);
            update_cost_rec(tree,i);
            
        }
    }
}

//****************************RRT Utilities******************************
//***********************************************************************
Eigen::Vector2d RRT::sample() {
    Eigen::Vector2d sampled_point;
    sampled_point[0] = x_dist(gen);
    sampled_point[1] = y_dist(gen);
    return sampled_point;
}
void RRT::add_node(std::vector<Node> &tree, Node node){
    node.cost = cost(tree,node);
    node.ind = tree.size();
    tree.push_back(node);
    if(node.parent > -1){
        tree[node.parent].children.insert(node.ind);
    }
    RRTstar(tree, tree.size()-1);
}
int RRT::nearest(std::vector<Node> &tree, Eigen::Vector2d &sampled_point) {
    int nearest_node = 0;
    double closest_dist = 100000;
    for(int i = 0; i < tree.size(); i++){
        if((tree[i].point - sampled_point).norm() < closest_dist){
            closest_dist = (tree[i].point - sampled_point).norm();
            nearest_node = i;
        }
    }
    return nearest_node;
}
Node RRT::steer(Node &nearest_node, int nearest_index, Eigen::Vector2d &sampled_point) {
    Node new_node;
    if((nearest_node.point - sampled_point).norm() < steer_dist){
        new_node.point = nearest_node.point + (sampled_point-nearest_node.point);
    }
    else new_node.point = nearest_node.point + (sampled_point-nearest_node.point).normalized()*steer_dist;
    new_node.parent = nearest_index;
    new_node.is_root = false;
    return new_node;
}
bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    //calculate estimate times for the segment
    Eigen::Vector2d nearest_node_map = car_frame_to_map_frame(nearest_node.point);
    Eigen::Vector2d new_node_map = car_frame_to_map_frame(new_node.point);
    bool occ_grid_collision = occ_grid_.is_occupied_line_metric(nearest_node_map, new_node_map);
    //  return occ_grid_collision;
    // ROS_INFO_STREAM("finished occ_grid collision checking");
    if(!predictor.prediction_ready) return occ_grid_collision;
    double start_time = nearest_node.cost/nominal_speed;
    double end_time = start_time + line_cost(nearest_node,new_node)/nominal_speed;
    //check for intersection
    // ROS_INFO_STREAM("checking dynamic intersection");
    bool dynam_intersection = check_dynamic_intersection(nearest_node_map, start_time, new_node_map, end_time,current_prediction,current_prediction_dt);
    // if(dynam_intersection) ROS_INFO_STREAM("dynamic_intersection");
    return occ_grid_collision || dynam_intersection;
}
//PREDICTION ADDITION
bool RRT::check_dynamic_intersection(Eigen::Vector2d start_pos_rrt, double start_time_rrt, Eigen::Vector2d end_pos_rrt, double end_time_rrt, std::vector<Eigen::Vector2d> oppo_pred, double oppo_dt){
    
    //closest intersection:
    //w(t) = distance between points as a function of time
    //w(t)_min = w0 - (w0.dot(u-v)/(u-v).dot(u-v))(u-v)
    //w0 = start_pos_eqo - start_of_pred_segment
    //u = ego_dir; v = pred_dir
    //http://geomalgorithms.com/a07-_distance.html?fbclid=IwAR1i4nnlL0uO8zUpu8D0ZmLXv8yrYubW-EVqwevKruK0Nbg5hpR7EOg5O-c
    
    //segment time is in rrt local time (0 at start of segment, end_time_rrt - start_time_rrt at end)
    double seg_time_length = end_time_rrt - start_time_rrt;

    //seg_start_time / end_time is a local tracker for the current segment being evaluated, not an indication of the time bounds
    double seg_start_time = 0;
    double seg_end_time = oppo_dt - std::fmod(start_time_rrt,oppo_dt);
    int pred_seg_ind = std::floor(start_time_rrt/oppo_dt);
    
    Eigen::Vector2d ego_vel = (end_pos_rrt-start_pos_rrt)/(seg_time_length);
    ego_vel = ego_vel * 4.5 / (ego_vel.norm());

    // ROS_INFO_STREAM("starting dynamic sweep");
    
    //while the end of the rrt segment has not been scanned
    while(seg_start_time < seg_time_length){
        //exit if end of prediction horizon is reached
        if(pred_seg_ind >= oppo_pred.size()-1) break;

        // ROS_INFO_STREAM("sweep 1");

        //start point of rrt segment update
        Eigen::Vector2d ego_seg_start_point = start_pos_rrt + seg_start_time*ego_vel;
        // ROS_INFO_STREAM("sweep 2");
        // ROS_INFO_STREAM(oppo_pred.size());
        // ROS_INFO_STREAM(pred_seg_ind);
        //find the opponent velocity of current prediction segment
        Eigen::Vector2d oppo_seg_vel = (oppo_pred[pred_seg_ind+1]-oppo_pred[pred_seg_ind])/oppo_dt;
        // ROS_INFO_STREAM("sweep 3");
        //find the opponent start position of segment
        Eigen::Vector2d oppo_seg_start_point = oppo_pred[pred_seg_ind] + std::fmod(start_time_rrt+seg_start_time,oppo_dt)*oppo_seg_vel;
        // ROS_INFO_STREAM("sweep 4");
        //find closest point for given time frame
        double min_dist = closest_distance_on_segment(ego_seg_start_point,oppo_seg_start_point,ego_vel,oppo_seg_vel,seg_end_time-seg_start_time);
        // ROS_INFO_STREAM("sweep 5");
        if(min_dist < dynamic_collision_distance) return true;

        //update to the next prediction segment, or cap at the end of the rrt segment
        // ROS_INFO_STREAM("sweep 6");
        pred_seg_ind++;
        seg_start_time = seg_end_time;
        seg_end_time = std::min(seg_start_time + oppo_dt, seg_time_length);
    }

    return false;

}

//PREDICTION ADDITION
double RRT::closest_distance_on_segment(Eigen::Vector2d P0, Eigen::Vector2d Q0,Eigen::Vector2d u, Eigen::Vector2d v, double time_length){
    //P0 and Q0 are starting points of segment, u and v are the velocities along line
    //w is a function for the distance between points on line at a given time
    Eigen::Vector2d w0 = P0 - Q0;
    Eigen::Vector2d w_dir = u-v;
    
    if(w_dir == Eigen::Vector2d::Zero()) return w0.norm();

    //solve for time of closest intersection
    double closest_time = w0.dot(w_dir)/(w_dir.dot(w_dir));
    if(closest_time > 0 && closest_time < time_length){
        //return distance between lines at time of the closest intersection
        return (w0+closest_time*w_dir).norm();
    }
    else{
        //conditions for if the closest intersection lies outside the given time horizon
        if(closest_time >= time_length){
            return (w0 + time_length*w_dir).norm();
        }
        else{
            return w0.norm();
        }
    }

}
//END PREDICTION ADDITION

//****************************RRT* Utilities*****************************
//***********************************************************************
double RRT::cost(std::vector<Node> &tree, Node &node) {
    if(node.parent == -1) return 0;
    return tree[node.parent].cost + line_cost(node,tree[node.parent]);
}
double RRT::line_cost(Node &n1, Node &n2) {
    return (n1.point - n2.point).norm();
}
void RRT::update_cost_rec(std::vector<Node> &tree, int ind){
    double new_cost = tree[tree[ind].parent].cost + line_cost(tree[tree[ind].parent],tree[ind]);
    if(tree[ind].cost == new_cost) return;
    tree[ind].cost = new_cost;
    for(int child:tree[ind].children){
        update_cost_rec(tree,child);
    }
}
std::vector<int> RRT::find_neighbors(std::vector<Node> &tree, int index, double dist){
   std::vector<int> ret;

   for(int i = 0; i<tree.size(); i++){
       if(i == index) continue;
       if((tree[i].point - tree[index].point).norm() < dist){
            ret.push_back(i);
        }
    }

    return ret;
}

//**************************PATH UTILITIES******************************
//**********************************************************************
std::vector<Node> RRT::find_path(std::vector<Node> &tree, Eigen::Vector2d goal) {    
    // Find the best path
    float best_dist = 10000;
    int best_by_dist = -1;
    float best_cost = 1000000;
    int best_by_cost = -1;

    for(int i = 0; i < tree.size(); i++){
        // Find closes path to the goal
        if((tree[i].point - goal).norm() < best_dist){
            best_by_dist = i;
            best_dist = (tree[i].point - goal).norm();
        }
        // To tie break sufficiently close paths, check the cost
        if((tree[i].point - goal).norm() < rrt_close_to_goal_dist){
            if (best_cost > tree[i].cost){
                best_cost = tree[i].cost;
                best_by_cost = i;
            }
        }
    }

    // Generate path from the tree
    std::vector<Node> found_path;
    int cur = best_by_cost == -1? best_by_dist:best_by_cost;
    int iter = 0;
    while(!tree[cur].is_root){
        found_path.push_back(tree[cur]);
        cur = tree[cur].parent;
        iter++;
        if(iter > 300){
            ROS_INFO_STREAM("FAILED AT BEST PATH");
            break;
        }
    }
    found_path.push_back(tree[cur]);
    found_path.reserve(found_path.size());
    return found_path ;
}
std::vector<Node> RRT::smooth_path(const std::vector<Node> &path){
    // path needs to at least be of size 2
    size_t num_waypoints = path.size();
    if(num_waypoints <= 2) return path;

    // Generate the new path
    std::vector<Node> smooth_path;
    smooth_path.push_back(path.front());
    
    for (size_t i = 0; i < num_waypoints; i++ )
    {

        Node start = path[i];
        size_t j = i + 1; // path needs to be at least size of 2
        if(j >= num_waypoints){
            smooth_path.push_back(start);
            return smooth_path;
        }
        Node end = path[j];

        while (!check_collision(start,end))
        {
            if (j >= num_waypoints - 1) // reached the final waypoint
            {

                smooth_path.push_back(end);

                return smooth_path;
            }
            j++;
            end = path[j];
        }

        j--; // need to go back to the last valid endpoint


        smooth_path.push_back(path[j]);

        i = j; // start at the endpoint
    }

}
void RRT::save_best_path(std::vector<Node> best_path){
    for(Node n:best_path){
        n.point = car_frame_to_map_frame(n.point);
        n.cost = 0;
    }
    previous_path = best_path;
    previous_path_init = true;
}
std::vector<Node> RRT::recover_best_path(){
    for(Node n:previous_path){
        n.point = map_frame_to_car_frame(n.point);
    }
    return previous_path;
}

//**************************GOAL/WAYPOINT METHODS***********************
//**********************************************************************
bool RRT::is_on_segment(Eigen::Vector2d ep1, Eigen::Vector2d ep2, Eigen::Vector2d pt){
    Eigen::Vector2d dir1 = ep1-ep2;
    Eigen::Vector2d pt1 = pt -ep2;
    Eigen::Vector2d dir2 = ep2-ep1;
    Eigen::Vector2d pt2 = pt-ep1;
    return pt1.dot(dir1)>0 && pt2.dot(dir2)>0;
}
Eigen::Vector2d RRT::find_best_valid_point(std::vector<Eigen::Vector2d> pt){  
    double best_val = -10000000;
    Eigen::Vector2d best_pt;
    
    //only need to check x as the local vehicle "most forward" possible waypoint
    for(int i = 0; i < pt.size(); i++){
        if(pt[i][0] > best_val){
            best_val = pt[i][0];
            best_pt = pt[i];
        }
    }
    
    return best_pt;
}
std::vector<Eigen::Vector2d> RRT::find_intersections(geometry_msgs::PoseStamped pose, double len){
    std::vector<Eigen::Vector2d> pts;
    
    for(int i = 0; i<waypoints.size(); i++){
        Eigen::Vector2d car_pt1 = world_to_car(pose,waypoints[i]);
        Eigen::Vector2d car_pt2 = world_to_car(pose,waypoints[(i+1)%waypoints.size()]);
        Eigen::Vector2d line = construct_mb_line(car_pt1,car_pt2);
        
        double m = line[0];
        double b = line[1];
        double det = 4*m*m*b*b - 4*(1+m*m)*(b*b-len*len);

        if(det < 0){
            continue;
        }
        else{
            Eigen::Vector2d int1;
            int1[0] = (-2*m*b + std::sqrt(det))/(2*(1+m*m));
            int1[1] = m*int1[0]+b;
            Eigen::Vector2d int2;
            int2[0] = (-2*m*b - std::sqrt(det))/(2*(1+m*m));
            int2[1] = m*int2[0]+b;
            if(is_on_segment(car_pt1, car_pt2, int1)){
                pts.push_back(int1);
            }
            if(is_on_segment(car_pt1, car_pt2, int2)){
                pts.push_back(int2);
                
            } 
        }
        
    }

    return pts;

} 
Eigen::Vector2d RRT::construct_mb_line(Eigen::Vector2d p1, Eigen::Vector2d p2){
    Eigen::Vector2d line;
    line[0] = (p2[1] - p1[1])/(p2[0] - p1[0]);
    line[1] = p1[1] - line[0]*p1[0];
    return line;
}
void RRT::initialize_points(){
    std::string logger_pack = ros::package::getPath("rrt_mpc");
    std::ifstream stream;
    std::string waypoints_path;
    nh_.getParam("waypoints", waypoints_path);
    ROS_INFO_STREAM("waypoints file path: ");
    ROS_INFO_STREAM(logger_pack+waypoints_path);
    stream.open(logger_pack + waypoints_path);
    if(!stream.good()){
        ROS_INFO_STREAM("No waypoint file found");
        return;
    }
    bool isX = true;
    double x;
    std::string temp;
    while(std::getline(stream,temp)){
        std::string temp2;
        std::stringstream line(temp);
        while(std::getline(line,temp2,',')){

        
            if(isX){
                x = std::stof(temp2);
                isX = false;
            }
            else{
                Eigen::Vector2d pt;
                pt[0] = x;
                pt[1] = std::stof(temp2);
                isX = true;
                waypoints.push_back(pt);
                

            }
        }

    }
    ROS_INFO_STREAM("LOADED POINTS: ");
    ROS_INFO_STREAM(waypoints.size());
}

//*************************TRANSFORMS************************************
//***********************************************************************
Eigen::Vector2d RRT::world_to_car(geometry_msgs::PoseStamped pose, Eigen::Vector2d pt){
    Eigen::Vector2d ret;

    double ang = 2*std::acos(pose.pose.orientation.w);
    double rot;
    if(ang == 0) rot = 0;
    else rot = ang*(pose.pose.orientation.z/std::sin(ang/2));

    double tempx = pt[0] - pose.pose.position.x;
    double tempy = pt[1] - pose.pose.position.y;
    ret[0] = std::cos(rot)*tempx + std::sin(rot)*tempy;
    ret[1] = -std::sin(rot)*tempx + std::cos(rot)*tempy;
    
    return ret;
}
Eigen::Vector2d RRT::transform_scan_to_map_frame(const geometry_msgs::PoseStamped &pose, const Eigen::Vector2d &wp){
    const double DIST_LASER_TO_BASE = 0.275;
    Eigen::Matrix2d R_car_in_map_frame = Eigen::Rotation2Dd(get_yaw(pose)).toRotationMatrix();
    Eigen::Vector2d d_car_in_map_frame(pose.pose.position.x, pose.pose.position.y);
    
    Eigen::Matrix3d H_car_in_map_frame;
    H_car_in_map_frame.topLeftCorner<2, 2>() = R_car_in_map_frame;
    H_car_in_map_frame.topRightCorner<2, 1>() = d_car_in_map_frame;
    H_car_in_map_frame.bottomLeftCorner<1, 2>().setZero();
    H_car_in_map_frame.bottomRightCorner<1, 1>().setOnes();
    
    Eigen::Vector3d wp_car_frame;
    wp_car_frame << wp.x()+DIST_LASER_TO_BASE, wp.y(), 1;
    Eigen::Vector2d wp_map_frame = (H_car_in_map_frame*wp_car_frame).head<2>();

    return wp_map_frame;
}
Eigen::Vector2d RRT::car_frame_to_map_frame(const Eigen::Vector2d &cp){
    Eigen::Matrix2d R_car_in_map_frame = Eigen::Rotation2Dd(get_yaw(most_recent_pose_)).toRotationMatrix();
    Eigen::Vector2d d_car_in_map_frame(most_recent_pose_.pose.position.x, most_recent_pose_.pose.position.y);
    
    Eigen::Matrix3d H_car_in_map_frame;
    H_car_in_map_frame.topLeftCorner<2, 2>() = R_car_in_map_frame;
    H_car_in_map_frame.topRightCorner<2, 1>() = d_car_in_map_frame;
    H_car_in_map_frame.bottomLeftCorner<1, 2>().setZero();
    H_car_in_map_frame.bottomRightCorner<1, 1>().setOnes();
    
    Eigen::Vector3d wp_car_frame;
    wp_car_frame << cp.x(), cp.y(), 1;
    Eigen::Vector2d wp_map_frame = (H_car_in_map_frame*wp_car_frame).head<2>();

    return wp_map_frame;
}
Eigen::Vector2d RRT::map_frame_to_car_frame(const Eigen::Vector2d &cp){
    Eigen::Matrix2d R_car_in_map_frame = Eigen::Rotation2Dd(get_yaw(most_recent_pose_)).toRotationMatrix();
    Eigen::Vector2d d_car_in_map_frame(most_recent_pose_.pose.position.x, most_recent_pose_.pose.position.y);

    Eigen::Matrix3d H_car_in_map_frame;
    H_car_in_map_frame.topLeftCorner<2, 2>() = R_car_in_map_frame;
    H_car_in_map_frame.topRightCorner<2, 1>() = d_car_in_map_frame;
    H_car_in_map_frame.bottomLeftCorner<1, 2>().setZero();
    H_car_in_map_frame.bottomRightCorner<1, 1>().setOnes();

    Eigen::Vector3d wp_map_frame;
    wp_map_frame << cp.x(), cp.y(), 1;
    Eigen::Vector2d wp_car_frame = (H_car_in_map_frame.inverse()*wp_map_frame).head<2>();

    return wp_car_frame;
}
double RRT::get_yaw(const geometry_msgs::PoseStamped &pose){
    tf2::Quaternion q(
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w);
    return tf2::impl::getYaw(q);
}

//************************VISUALIZATION**********************************
//***********************************************************************
void RRT::visualize_best_path(std::vector<Node> nodes){
    visualization_msgs::Marker m;
    m.action = m.ADD;
    m.type = m.LINE_LIST;
    m.header.frame_id = "ego_racecar/base_link";
    m.scale.x = .1f;
    geometry_msgs::Quaternion q;
    q.w = 1;
    m.pose.orientation =q;
    std_msgs::ColorRGBA c;
    c.g = 1;
    c.a = 1;
    m.color = c;
    for(int i = 0; i <  nodes.size()-1; i++){
        if(nodes[i].is_root) continue;
        
        geometry_msgs::Point p1;
        p1.x = nodes[i].point[0];
        p1.y = nodes[i].point[1];
        geometry_msgs::Point p2;
        p2.x = nodes[i+1].point[0];
        p2.y = nodes[i+1].point[1];

        m.points.push_back(p1);
        m.points.push_back(p2);
        m.colors.push_back(c);
        m.colors.push_back(c);
        
        //m.colors.push_back(c);
        
    }
    best_line_pub.publish(m);

}
void RRT::visualize_tree(std::vector<Node> tree){
    visualization_msgs::Marker m;
    m.action = m.ADD;
    m.type = m.LINE_LIST;
    m.header.frame_id = "ego_racecar/base_link";
    m.scale.x = .02f;
    geometry_msgs::Quaternion q;
    q.w = 1;
    m.pose.orientation =q;
    std_msgs::ColorRGBA c;
    c.b = 1;
    c.r = 0;
    c.g = 0;
    c.a = 1;
    m.color = c;
    for(int i = 0; i < tree.size(); i++){
        if(tree[i].is_root) continue;
        
        geometry_msgs::Point p1;
        p1.x = tree[i].point[0];
        p1.y = tree[i].point[1];
        geometry_msgs::Point p2;
        p2.x = tree[tree[i].parent].point[0];
        p2.y = tree[tree[i].parent].point[1];

        m.points.push_back(p1);
        m.points.push_back(p2);
        m.colors.push_back(c);
        m.colors.push_back(c);
        
        //m.colors.push_back(c);
        
    }
    tree_pub.publish(m);
}
void RRT::visualize_waypoints(){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    for(int i = 0; i < waypoints.size()+1; i++){
        geometry_msgs::Point msg_pt;
        msg_pt.x = waypoints[i%waypoints.size()][0];
        msg_pt.y = waypoints[i%waypoints.size()][1];
        marker.points.push_back(msg_pt);
    }
    
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1;
    marker.color.a = 1;
    marker.scale.x = .1;
    
    line_pub.publish(marker);
}
