// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the node definition for RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf


#include "rrt/rrt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "team2_milestone3_node");
    ros::NodeHandle nh("/team2_milestone3");
    RRT rrt(nh);
    ros::spin();
    return 0;
}
