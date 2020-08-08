// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>

// Standard
#include <vector>
#include "eigen3/Eigen/Core"


class OccGrid
{
public:
    // Constructor
    OccGrid() = default;
    OccGrid(const nav_msgs::OccupancyGrid &map_msg);

    // Grid Updates
    void clear_dynamic_grid();
    void set_dynamic_grid_point(const Eigen::Vector2d &p);

    // check occupied
    bool is_occupied_index(const Eigen::Vector2i &idx) const;
    bool is_occupied_index(const int &ix, const int &idx) const;
    bool is_occupied_metric(const double &x, const double &y) const;
    bool is_occupied_metric(const Eigen::Vector2d &metric) const;

    bool is_occupied_line_metric(const Eigen::Vector2d &endpoint1, const Eigen::Vector2d &endpoint2 );
    bool is_occupied_line_index(const Eigen::Vector2i &endpoint1, const Eigen::Vector2i &endpoint2 );

    // visualization
    visualization_msgs::Marker get_vis_occ_grid_msg() const;

    // utilities
    std::vector<Eigen::Vector2i> bresenham(Eigen::Vector2i p1, Eigen::Vector2i p2);
    void add_pt_bresenham(std::vector<Eigen::Vector2i> &line,  int x, int y, bool steep, bool x_ref, bool y_ref);
    Eigen::Vector2i metric_to_index(const Eigen::Vector2d &metric) const;
    Eigen::Vector2d index_to_metric(const Eigen::Vector2i &idx) const;

private:
    Eigen::MatrixXd occ_grid_;
    Eigen::MatrixXd occ_grid_dynamic_;
    int map_width_, map_height_;
    double map_resolution_;
    Eigen::Vector2d origin_metric_;
    int padding_x_;
    int padding_y_;
};