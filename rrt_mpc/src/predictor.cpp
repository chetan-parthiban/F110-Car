#include "predictions/predictor.h"

Predictor::Predictor(ros::NodeHandle &nh, PurePursuit &pp): nh_(nh), controller(pp){
    std::string opponent_topic, vis_topic;
    nh_.getParam("opponent_topic", opponent_topic);
    nh_.getParam("vis_topic", vis_topic);

    nh_.getParam("dt", dt);
    nh_.getParam("steps", steps);
    nh_.getParam("linear_weight_decay", linear_weight_decay);
    nh_.getParam("wheelbase", wheelbase);
    nh_.getParam("max_speed", max_speed);
    nh_.getParam("max_accel", max_accel);
    nh_.getParam("max_decel", max_decel);
    nh_.getParam("do_visualization", do_visualization);

    // Create ROS subscribers and publishers
    odom_sub_  = nh_.subscribe(opponent_topic, 1, &Predictor::odom_callback, this);
    vis_pub_   = nh_.advertise<visualization_msgs::Marker>(vis_topic, 100);
}

void Predictor::odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
    // ROS_INFO_STREAM("predictor odom_callback");
    current_pos = msg;

    prediction_ready = true;
}

std::vector<Eigen::VectorXd> Predictor::get_path(){
    return get_path(current_pos);
}

std::vector<Eigen::VectorXd> Predictor::get_path(const nav_msgs::Odometry::ConstPtr &msg){
    // initialize enemy position
    double opp_x = msg->pose.pose.position.x;
    double opp_y = msg->pose.pose.position.y;
    double opp_yaw = get_yaw(msg);
    double opp_v = msg->twist.twist.linear.x;
    Eigen::VectorXd opp_pos(4);
    opp_pos << opp_x, opp_y, opp_yaw, opp_v;

    // for each step update position and push back
    std::vector<Eigen::VectorXd> opp_path;
    opp_path.push_back(opp_pos);
    for (int i = 0; i < steps; i++) {

        opp_pos = Predictor::update(opp_pos, i);
        opp_path.push_back(opp_pos);
    }
    // // visualize if we want
    if (do_visualization) {visualize_path(opp_path); }
    return opp_path;
}

Eigen::VectorXd Predictor::update(Eigen::VectorXd &opp_pos, int step){
    
    double linear_weight = std::pow(linear_weight_decay, step);
    return linear_weight * update_linear(opp_pos) + (1 - linear_weight) * update_long(opp_pos);
}

Eigen::VectorXd Predictor::update_linear(Eigen::VectorXd &opp_pos){
    double dx = opp_pos(3) * std::cos(opp_pos(2));
    double dy = opp_pos(3) * std::sin(opp_pos(2));

    double x = opp_pos(0) + dx * dt;
    double y = opp_pos(1) + dy * dt;

    Eigen::VectorXd updated(4);
    updated << x, y, opp_pos(2), opp_pos(3);
    return updated;
}

Eigen::VectorXd Predictor::update_long(Eigen::VectorXd &opp_pos){
    // get planned control
    
    Eigen::VectorXd control = controller.get_control(opp_pos);
    double steer = control(0);
    double v = control(1);
    // double beta = std::atan(std::tan(steer) * 0.5);
    

    double dx = opp_pos(3) * std::cos(opp_pos(2));
    double dy = opp_pos(3) * std::sin(opp_pos(2));
    double dtheta = opp_pos(3) / wheelbase * std::tan(steer);
    double accel = std::min(max_accel, compute_accel(opp_pos(3), v));

    double x = opp_pos(0) + dx * dt;
    double y = opp_pos(1) + dy * dt;
    double theta = opp_pos(2) + dtheta * dt;
    double vel = opp_pos(3) + accel * dt;

    Eigen::VectorXd updated(4);
    updated << x, y, theta, opp_pos(3);
    return updated;
}

double Predictor::compute_accel(double v, double v_des) {
    double dif = v_des - v;
    double accel;
    if ((v > 0) && (v_des >= 0)) {
        double kp = 2.0 * max_accel/max_speed;
        accel = kp * dif;
    } else if ((v > 0) && (v_des < 0)) {
        accel = -max_decel;
    } else if ((v < 0) && (v_des > 0)) {
        accel = max_decel;
    } else {
        double kp = 2.0 * max_accel/max_speed;
        accel = kp * dif;
    }
    return accel;
}

void Predictor::visualize_path(std::vector<Eigen::VectorXd> &points){
    visualization_msgs::Marker m;
    m.action = m.ADD;
    m.type = m.LINE_LIST;
    m.header.frame_id = "/map";
    m.scale.x = .1f;

    geometry_msgs::Quaternion q;
    q.w = 1;
    m.pose.orientation = q;

    std_msgs::ColorRGBA c;
    c.g = 1;
    c.a = 1;
    m.color = c;

    for(int i = 0; i <  points.size()-1; i++){        
        geometry_msgs::Point p1;
        p1.x = points[i].x();
        p1.y = points[i].y();
        geometry_msgs::Point p2;
        p2.x = points[i+1].x();
        p2.y = points[i+1].y();

        m.points.push_back(p1);
        m.points.push_back(p2);
        m.colors.push_back(c);
        m.colors.push_back(c);    
    }

    vis_pub_.publish(m);
    // ros::Duration(1).sleep();
}

double get_yaw(const nav_msgs::Odometry::ConstPtr &msg){
    tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
    return tf2::impl::getYaw(q);
}

//access methods
    double Predictor::get_dt(){
        return dt;
    }