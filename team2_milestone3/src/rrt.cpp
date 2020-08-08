// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    
    ROS_INFO_STREAM("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic, map_topic, occgrid_topic, tree_topic, test_topic, line_topic,nav_topic,best_line_topic;
    if(nh_.getParam("pose_topic", pose_topic)){
        ROS_INFO_STREAM("pose topic loaded");
    }
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("map_topic", map_topic);
    nh_.getParam("viz_topic",occgrid_topic);
    nh_.getParam("min_scan_dist", min_scan_dist);
    nh_.getParam("test_topic",test_topic);
    nh_.getParam("tree_topic", tree_topic);
    nh_.getParam("line_topic", line_topic);
    nh_.getParam("nav_topic",nav_topic);
    nh_.getParam("best_line_topic",best_line_topic);


    
    //these params can be but into yaml 
    nh_.getParam("lidar_offset",lidar_offset);
    nh_.getParam("x_sample",x_sample);
    nh_.getParam("y_sample",y_sample);
    
    nh_.getParam("steer_dist",steer_dist);
    nh_.getParam("RRTs_connection_dist",RRTs_connection_dist);
    nh_.getParam("rrt_close_to_goal",rrt_close_to_goal_dist);

    nh_.getParam("tree_nodes",tree_nodes);
    nh_.getParam("L",car_intersection_radius);
    nh_.getParam("max_iter",max_iter);

    
    nh_.getParam("speed",speed);
    nh_.getParam("slowdown",slowdown);

    
    x_dist = std::uniform_real_distribution<>(lidar_offset,x_sample);
    y_dist = std::uniform_real_distribution<>(-y_sample,y_sample);
    gen = std::mt19937(ros::Time().nsec);
    
    //LOAD WAYPOINTS
    initialize_points();

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need

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
    ROS_INFO_STREAM(map_msg.data.size());
    occ_grid_ = OccGrid(map_msg);
    
    ROS_INFO_STREAM("BUILT OCCGRID");
    
    //pf_sub_ = nh_.subscribe(pose_topic, 1, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 1, &RRT::scan_callback, this);
    test_sub = nh_.subscribe(test_topic, 1, &RRT::test_callback,this);



    occgrid_pub = nh_.advertise<visualization_msgs::Marker>(occgrid_topic,1);
    tree_pub = nh_.advertise<visualization_msgs::Marker>(tree_topic,1);
    line_pub = nh_.advertise<visualization_msgs::Marker>(line_topic,1);
    best_line_pub = nh_.advertise<visualization_msgs::Marker>(best_line_topic,1);
    nav_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(nav_topic,1);

    ROS_INFO_STREAM("Finished connecting to topics");

}

//**************************CALLBACKS*****************************
//****************************************************************

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    //ROS_INFO_STREAM_THROTTLE(.5f,"scan_callback");
    
    if(!pose_init) return;
    
    occ_grid_.clear_dynamic_grid();
    for (size_t i = 0; i < scan_msg->ranges.size(); i++)
    {
        double r = scan_msg->ranges[i];
        if (r >= scan_msg->range_min && r <= scan_msg->range_max)
        {
            double theta = scan_msg->angle_min + i*scan_msg->angle_increment;
            Eigen::Vector2d scan_xy_car_frame;
            scan_xy_car_frame << r*std::cos(theta), r*std::sin(theta);
            Eigen::Vector2d scan_xy_map_frame = transform_scan_to_map_frame(most_recent_pose_, scan_xy_car_frame);
            occ_grid_.set_dynamic_grid_point(scan_xy_map_frame);
        }
    }
    
    
    //ROS_INFO_STREAM_THROTTLE(.5f,"finishing scan callback");
    occgrid_pub.publish(occ_grid_.get_vis_occ_grid_msg()); 

    
}


void RRT::test_callback(const nav_msgs::Odometry &pose_msg){
    geometry_msgs::PoseStamped geo_pose;
    geo_pose.pose = pose_msg.pose.pose;
    geo_pose.pose.orientation = pose_msg.pose.pose.orientation;
    most_recent_pose_ = geo_pose;
    
    pose_init = true;
    visualize_waypoints();
    //ROS_INFO_STREAM_THROTTLE(.5f,"testing for intersections");
    std::vector<Eigen::Vector2d> interscetions = find_intersections(most_recent_pose_,car_intersection_radius);
    if(interscetions.size() == 0){
        ROS_INFO_STREAM_THROTTLE(.1f,"NO INTERSECTIONS");

        return;
        //find nearest waypoint

    }
    else{
        current_goal = find_best_valid_point( interscetions);  
        
        //ROS_INFO_STREAM(goal);
        //visualize_target(target);
        //ROS_INFO_STREAM_THROTTLE(.5f,"running");
        run_RRT();
    }
    
    
}

void RRT::pf_callback(const geometry_msgs::PoseStamped &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    //ROS_INFO_STREAM_THROTTLE(.5f,"pose_callback");
    most_recent_pose_ = pose_msg;
    
    pose_init = true;


    visualize_waypoints();
    std::vector<Eigen::Vector2d> interscetions = find_intersections(most_recent_pose_,car_intersection_radius);
    if(interscetions.size() == 0){
        ROS_INFO_STREAM_THROTTLE(.1f,"NO INTERSECTIONS");

        return;
        //find nearest waypoint

    }
    else{
        current_goal = find_best_valid_point( interscetions);  
        //ROS_INFO_STREAM(goal);

        //visualize_target(target);
        run_RRT();
    }
    


    // path found as Path message

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

//****************************************RRT METHODS*************************************
//*****************************************************************************************
void RRT::run_RRT(){
    
    if(!pose_init) return;
    
    // tree as std::vector
    adjacencies.clear();
    //tried adjacencies grid to speed up finding neighbors but is currently not in use
    
    std::vector<Node> tree;
    /*
        for(int i = 0; i < x_desc; i++){
        adjacencies.push_back(std::vector<std::vector<int>>(y_desc));
    }
    */

    //set root as first node
    Node root;
    root.point = Eigen::Vector2d::Zero();
    root.point[0] = lidar_offset;
    root.is_root = true;
    root.cost = 0;
    root.parent = -1;
    add_node(tree,root);


    //grow tree to 300 nodes (abstract to parameter for tuning)
    int c = 0;
    while(tree.size() < tree_nodes && c < max_iter){
        //ROS_INFO_STREAM("ITERATION");
        //ROS_INFO_STREAM(i);
        Eigen::Vector2d pt = sample();
        int par = nearest(tree, pt);
        //ROS_INFO_STREAM("Found parent");
        Node n = steer(tree[par],par,pt);
        if(!check_collision(tree[par],n)){
           add_node(tree,n);
            
        }
        c++;
        
    }

    if(previous_path_init){
        for(Node p_node: recover_best_path()){
            int par = nearest(tree, p_node.point);
        //ROS_INFO_STREAM("Found parent");
            Node n = steer(tree[par],par,p_node.point);
            if(!check_collision(tree[par],n)){
                add_node(tree,n);
                
            }
        }
        
    }
    //choice of visualization
    

    //find best path given current goal and tree
    
    std::vector<Node> local_path = find_path(tree,current_goal);

    if(local_path.size() < 2) {
        // ROS_INFO_STREAM("Tree_failed");
        ackermann_msgs::AckermannDriveStamped drive_msg;
        //********************This will need to change for dynamic obstacles******************************************************
        drive_msg.drive.steering_angle = get_steering_angle(recover_best_path()[previous_path.size()-2].point);
        // ROS_INFO_STREAM_THROTTLE(.5f,local_path[local_path.size()-2].point);
        // ROS_INFO_STREAM_THROTTLE(.5f,drive_msg.drive.steering_angle);
        if(std::isnan(drive_msg.drive.steering_angle)) drive_msg.drive.steering_angle = 0;
        drive_msg.drive.speed = speed - slowdown*speed*std::abs(drive_msg.drive.steering_angle);
        // ROS_INFO_STREAM(drive_msg.drive.speed);
        // ROS_INFO_STREAM(drive_msg.drive.steering_angle);
        nav_pub.publish(drive_msg);
        return;
    }
    save_best_path(local_path);
    local_path = smooth_path(local_path);
    visualize_tree(tree);
    visualize_best_path(local_path);
    
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.drive.steering_angle = get_steering_angle(local_path[local_path.size()-2].point);
    // ROS_INFO_STREAM_THROTTLE(.5f,local_path[local_path.size()-2].point);
    // ROS_INFO_STREAM_THROTTLE(.5f,drive_msg.drive.steering_angle);
    drive_msg.drive.speed = speed - slowdown*speed*std::abs(drive_msg.drive.steering_angle);
    // ROS_INFO_STREAM("Tree_success");
    // ROS_INFO_STREAM(drive_msg.drive.speed);
    // ROS_INFO_STREAM(drive_msg.drive.steering_angle);
    nav_pub.publish(drive_msg);
    

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
        // ROS_INFO_STREAM("recovering point:");
        // ROS_INFO_STREAM(map_frame_to_car_frame(n.point));
        n.point = map_frame_to_car_frame(n.point);
    }
    return previous_path;
}

double RRT::get_steering_angle(Eigen::Vector2d targ){
    Eigen::Vector2d forward = {1,0};
    
    targ = targ.normalized();
    double sign = targ[1]/std::abs(targ[1]);
    double ang = std::acos(targ.dot(forward))*sign;
    ang = std::min(ang, .45);
    ang = std::max(ang, -.45);


    //pure pursuit
    // double l = (targ).norm();
    // double ang = 1.5*targ[1]/(l*l);
    // ang = std::min(ang, .4f);
    // ang = std::max(ang, -.4f);


    return ang;

}
void RRT::add_node(std::vector<Node> &tree, Node node){
    node.cost = cost(tree,node);
    node.ind = tree.size();
    tree.push_back(node);
    if(node.parent > -1){
        tree[node.parent].children.insert(node.ind);
    }
    
    
    //RRTs_connection_dist = std::sqrt(std::log(tree.size())/tree.size());
    //update_adjacencies(tree.size()-1,node.point);
    //RRT* time

    RRTstar(tree,tree.size()-1);

}





//attempted to use adjacencies but went with full search for the time being
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


Eigen::Vector2d RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    Eigen::Vector2d sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    sampled_point[0] = x_dist(gen);
    sampled_point[1] = y_dist(gen);
    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, Eigen::Vector2d &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double closest_dist = 100000;
    // TODO: fill in this method
    for(int i = 0; i < tree.size(); i++){
        if((tree[i].point - sampled_point).norm() < closest_dist){
            closest_dist = (tree[i].point - sampled_point).norm();
            nearest_node = i;
        }
    }

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, int nearest_index, Eigen::Vector2d &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    // TODO: fill in this method
    if((nearest_node.point - sampled_point).norm() < steer_dist){
        new_node.point = nearest_node.point + (sampled_point-nearest_node.point);
    }
    else new_node.point = nearest_node.point + (sampled_point-nearest_node.point).normalized()*steer_dist;
    //ROS_INFO_STREAM("Creating new node");
    new_node.parent = nearest_index;
    new_node.is_root = false;
    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    
    // TODO: fill in this method
    

    
    return occ_grid_.is_occupied_line_metric(car_frame_to_map_frame(nearest_node.point), car_frame_to_map_frame(new_node.point));
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Eigen::Vector2d goal) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<Node> found_path;
    // TODO: fill in this method
    double best_dist = 10000;
    int best_by_dist = -1;
    double best_cost = 1000000;
    int best_by_cost = -1;
    for(int i = 0; i < tree.size(); i++){
        if((tree[i].point - goal).norm() < best_dist){
            best_by_dist = i;
            best_dist = (tree[i].point - goal).norm();
            
        }
        if((tree[i].point - goal).norm() < rrt_close_to_goal_dist){
            if(best_cost > tree[i].cost){
                best_cost = tree[i].cost;
                best_by_cost = i;
            }
        }
    }

    int cur = best_by_cost == -1? best_by_dist:best_by_cost;
    if(best_by_cost != -1){
        // ROS_INFO_STREAM("Chose best by cost");
    }
    int iter = 0;
    while(!tree[cur].is_root){
        found_path.push_back(tree[cur]);
        cur = tree[cur].parent;
        iter++;
        if(iter > 300){
            // ROS_INFO_STREAM("FAILED AT BEST PATH");
            break;
        }
    }
    found_path.push_back(tree[cur]);
    found_path.reserve(found_path.size());
    return found_path ;
}

std::vector<Node> RRT::smooth_path(const std::vector<Node> &path)
{
    size_t num_waypoints = path.size();
    if(num_waypoints <= 2) return path;
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

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node
    if(node.parent == -1) return 0;
    double cost = tree[node.parent].cost + line_cost(node,tree[node.parent]);
    // TODO: fill in this method
    

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = (n1.point - n2.point).norm();
    // TODO: fill in this method

    return cost;
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
            if(check_collision(tree[ind],tree[i])) continue;
            tree[tree[i].parent].children.erase(i);
            tree[i].parent = ind;
            tree[tree[i].parent].children.insert(i);
            tree[i].cost = tree[ind].cost + line_cost(tree[ind],tree[i]);
            update_cost_rec(tree,i);
            
        }
    }
}

void RRT::update_cost_rec(std::vector<Node> &tree, int ind){
    double new_cost = tree[tree[ind].parent].cost + line_cost(tree[tree[ind].parent],tree[ind]);
    if(tree[ind].cost == new_cost) return;
    tree[ind].cost = new_cost;
    for(int child:tree[ind].children){
        update_cost_rec(tree,child);
    }
}




//****************************************PATH METHODS**************************************************
//******************************************************************************************************


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

//use intersections between possible line hits and a surrounding circle around car to find points on current path
std::vector<Eigen::Vector2d> RRT::find_intersections(geometry_msgs::PoseStamped pose, double len){
    std::vector<Eigen::Vector2d> pts;
    
    
    
    for(int i = 0; i<waypoints.size(); i++){
        
        Eigen::Vector2d car_pt1 = world_to_car(pose,waypoints[i]);
        Eigen::Vector2d car_pt2 = world_to_car(pose,waypoints[(i+1)%waypoints.size()]);
        //ROS_INFO_STREAM_THROTTLE(.1f,car_pt1.x);
        //ROS_INFO_STREAM_THROTTLE(.1f,car_pt1.y);
        //ROS_INFO_STREAM_THROTTLE(.1f,car_pt2.x);
        //ROS_INFO_STREAM_THROTTLE(.1f,car_pt2.y);
        Eigen::Vector2d line = construct_mb_line(car_pt1,car_pt2);
        
        double m = line[0];
        double b = line[1];
        double det = 4*m*m*b*b - 4*(1+m*m)*(b*b-len*len);
        //ROS_INFO_STREAM_THROTTLE(.1f,m);
        //ROS_INFO_STREAM(line.y);
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
    
    //ROS_INFO_STREAM("finding intersections");
    return pts;

} 

Eigen::Vector2d RRT::construct_mb_line(Eigen::Vector2d p1, Eigen::Vector2d p2){
    Eigen::Vector2d line;
    line[0] = (p2[1] - p1[1])/(p2[0] - p1[0]);
    line[1] = p1[1] - line[0]*p1[0];
    return line;
}

Eigen::Vector2d RRT::get_current_goal(geometry_msgs::PoseStamped pose){
    ROS_INFO_STREAM("SHOULD NOT BE CALLED");
    //std::vector<Eigen::Vector2d> interscetions = find_intersections(pose,3);
    //ROS_INFO_STREAM("Intersections found: ");
    //ROS_INFO_STREAM(interscetions.size());
    //if(interscetions.size() == 0){
      //  ROS_INFO_STREAM_THROTTLE(.1f,"NO INTERSECTIONS");

        //return Eigen::Vector2d::Zero();
        //find nearest waypoint

    //}
    //else{
        //return find_best_valid_point( interscetions);  
    
    //}
    return Eigen::Vector2d::Zero();
}

void RRT::initialize_points(){
    std::string logger_pack = ros::package::getPath("team2_milestone3");
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

Eigen::Vector2d RRT::transform_scan_to_map_frame(const geometry_msgs::PoseStamped &pose,
                                          const Eigen::Vector2d &wp)
{
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

double RRT::get_yaw(const geometry_msgs::PoseStamped &pose)
{
    tf2::Quaternion q(
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w);
    return tf2::impl::getYaw(q);
}