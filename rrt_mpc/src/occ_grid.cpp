#include "rrt/occ_grid.h"
#include <geometry_msgs/Pose.h>

// Constructor
OccGrid::OccGrid(const nav_msgs::OccupancyGrid &map_msg)
{
    // Initialize main parameters
    map_width_ = map_msg.info.width;
    map_height_ = map_msg.info.height;
    
    origin_metric_[0] = -7.801;
    origin_metric_[1] = -16.388;
    map_resolution_ = map_msg.info.resolution;
    ROS_INFO_STREAM("OccGrid: set resolution");

    // Get the static grid info
    std::vector<double> map_vector(map_msg.data.size());
    for (size_t i = 0; i < map_msg.data.size(); i++)
    {
        int8_t d = map_msg.data[i];
        if (0 <= d and d <= 80)
        {
            map_vector[i] = 0; // Free
        }
        else
        {
            map_vector[i] = 1; // Occupied
        }
    }
    ROS_INFO_STREAM("setup vector");  

    // Initialize the occupancy grid and dynamic grid
    occ_grid_ = Eigen::MatrixXd(map_width_, map_height_);
    occ_grid_dynamic_ = Eigen::MatrixXd(map_width_, map_height_);
    clear_dynamic_grid();
    Eigen::MatrixXd occ_grid_raw = Eigen::Map<Eigen::MatrixXd>(map_vector.data(), map_width_, map_height_);
    padding_x_ = static_cast<int>(0.5*0.5/map_resolution_); // wheelbase
    padding_y_ = static_cast<int>(0.5*0.5/map_resolution_); // car width
    int r = 4*padding_x_+1;
    int c = 4*padding_y_+1;
    
    for (size_t x = 0; x < map_width_; x++)
    {
        for (size_t y = 0; y < map_height_; y++)
        {
            int ix = x-padding_x_;
            int iy = y-padding_y_;

            if (ix < 0)
            {
                ix = 0;
            }
            if (ix + r > map_width_)
            {
                ix = map_width_ - r;
            }
            
            if (iy < 0)
            {
                iy = 0;
            }
            if (iy + c > map_height_)
            {
                iy = map_height_ - c;
            }

            
            occ_grid_(x, y) = occ_grid_raw.block(ix, iy, r, c).any();
        }
    }

    ROS_INFO_STREAM("finished occgrid init");
}

// Update Grid
void OccGrid::clear_dynamic_grid()
{
    occ_grid_dynamic_.setZero();
}
void OccGrid::set_dynamic_grid_point(const Eigen::Vector2d &p)
{
    Eigen::Vector2i idx = metric_to_index(p);

    int ix = idx.x() - padding_x_-1; // extra padding for dynamic occ grid
    int iy = idx.y() - padding_y_-1;
    int r = 2*padding_x_+2;
    int c = 2*padding_y_+2;

    if (ix < 0)
    {
        ix = 0;
    }
    if (ix + r > map_width_)
    {
        ix = map_width_ - r;
    }
    
    if (iy < 0)
    {
        iy = 0;
    }
    if (iy + c > map_height_)
    {
        iy = map_height_ - c;
    }
    occ_grid_dynamic_.block(ix, iy, r, c).setOnes();
    occ_grid_dynamic_(idx.x(), idx.y()) = 1;
}


// Check Occupied
bool OccGrid::is_occupied_index(const Eigen::Vector2i &idx) const
{
    return occ_grid_(idx.x(), idx.y()) != 0 || occ_grid_dynamic_(idx.x(), idx.y()) != 0;
}
bool OccGrid::is_occupied_index(const int &ix, const int &iy) const
{
    return occ_grid_(ix, iy) != 0 || occ_grid_dynamic_(ix, iy) != 0;
}
bool OccGrid::is_occupied_metric(const double &x, const double &y) const
{
    Eigen::Vector2d m;
    m << x, y;
    return is_occupied_metric(m);
}
bool OccGrid::is_occupied_metric(const Eigen::Vector2d &metric) const
{
    return is_occupied_index(metric_to_index(metric));
}
bool OccGrid::is_occupied_line_metric(const Eigen::Vector2d &endpoint1, const Eigen::Vector2d &endpoint2 ){
    std::vector<Eigen::Vector2i> line = bresenham(metric_to_index(endpoint1),metric_to_index(endpoint2));
    for(Eigen::Vector2i p: line){
        if(is_occupied_index(p)) return true;
    }
    return false;
}
bool OccGrid::is_occupied_line_index(const Eigen::Vector2i &endpoint1, const Eigen::Vector2i &endpoint2 ){
    std::vector<Eigen::Vector2i> line = bresenham(endpoint1,endpoint2);
    for(Eigen::Vector2i p: line){
        if(is_occupied_index(p)) return true;
    }
    return false;
}

// Visualization
visualization_msgs::Marker OccGrid::get_vis_occ_grid_msg() const{
    visualization_msgs::Marker msg;
    double msg_scale = map_resolution_*0.9;
    msg.ns = "OccGrid";
    msg.id = 1;
    msg.type = msg.CUBE_LIST;
    msg.action = msg.ADD;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = msg_scale;
    msg.scale.y = msg_scale;
    msg.scale.z = msg_scale;
    msg.color.a = 0.6; // Don't forget to set the alpha!
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;

    Eigen::MatrixXd og_static_dynamic = occ_grid_.array() + occ_grid_dynamic_.array();
    
    
    for (size_t x = 0; x < map_width_; x++)
    {
        for (size_t y = 0; y < map_height_; y++)
        {
            if (og_static_dynamic(x, y))
            {
                geometry_msgs::Point p;
                Eigen::Vector2d metric = index_to_metric(Eigen::Vector2i(x, y));
                p.x = metric.x();
                p.y = metric.y();
                p.z = 0;
                msg.points.push_back(p);
            }
        }
    }
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time();
    return msg;
}

// Utilities
std::vector<Eigen::Vector2i> OccGrid::bresenham(Eigen::Vector2i p1, Eigen::Vector2i p2){
    //finds all cells between p1 and p2 in order from p1 to p2
    std::vector<Eigen::Vector2i> ret;
    if(p1 == p2){
        ret.push_back(p1);
        return ret;
    }
    int dx = p2[0] - p1[0];
    int dy = p2[1] - p1[1];
    
    
    bool y_reflect = dy<0;
    //flip y
    if(y_reflect){
        p1[1] = -p1[1];
        p2[1] = -p2[1];
    }

    bool x_reflect = dx <0;
    if(x_reflect){
        p1[0] = -p1[0];
        p2[0] = -p2[0];
    }
    //flip x

    bool steep = abs(dy) > abs(dx);



    //ROS_INFO_STREAM_THROTTLE(0.5f,"steep: ");
    //ROS_INFO_STREAM_THROTTLE(0.5f,steep);
    //ROS_INFO_STREAM_THROTTLE(0.5f,"x ref: ");
    //ROS_INFO_STREAM_THROTTLE(0.5f,x_reflect);
    //ROS_INFO_STREAM_THROTTLE(0.5f,"y ref: ");
    //ROS_INFO_STREAM_THROTTLE(0.5f,y_reflect);


    if(steep){
        p1.reverseInPlace();
        p2.reverseInPlace();
    }
    dx = p2[0] - p1[0];
    dy = p2[1] - p1[1];

    int x,y,xend,yend,p,Dy2,DxDy2;
    x = p1[0];
    y = p1[1];
    xend = p2[0];
    yend = p2[1];
    p = 2*dy - dx;
    Dy2 = 2*dy;
    DxDy2 = 2*dy - 2*dx;
    //ROS_INFO_STREAM_THROTTLE(0.5f,"dx");
    //ROS_INFO_STREAM_THROTTLE(0.5f,dx);
    //ROS_INFO_STREAM_THROTTLE(0.5f,"dy");
    //ROS_INFO_STREAM_THROTTLE(0.5f,dy);
    add_pt_bresenham(ret,x,y,steep,x_reflect,y_reflect);
    while(x < xend){
        x++;
        if(p>=0){
            y++;
            p+=DxDy2;
        }
        else{
            p+=Dy2;
        }
        add_pt_bresenham(ret,x,y,steep,x_reflect,y_reflect);
    }
    //add_pt(ret,xend,yend,steep,x_reflect,y_reflect);
    
    return ret;
    
}
void OccGrid::add_pt_bresenham(std::vector<Eigen::Vector2i> &line,  int x, int y, bool steep, bool x_ref, bool y_ref){
    Eigen::Vector2i ret = {x,y};
    if(steep){
        ret.reverseInPlace();
        
    }
    if(x_ref){
        ret[0] = -ret[0];
    }
    if(y_ref){
        ret[1] = -ret[1];
    }

    line.push_back(ret);
}
Eigen::Vector2i OccGrid::metric_to_index(const Eigen::Vector2d &metric) const{
    return ((metric-origin_metric_)/map_resolution_).cast<int>();
}
Eigen::Vector2d OccGrid::index_to_metric(const Eigen::Vector2i &idx) const{
    return map_resolution_ * (idx).cast<double>() + origin_metric_;
}