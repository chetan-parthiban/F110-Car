#include "mpc/mpc_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "team2_mpc_node");
    ros::NodeHandle nh("/mpc");
    MPCManager mpc_manager(nh);
    ros::spin();
    return 0;
}
