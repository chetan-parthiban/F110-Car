#include "mpc/mpc_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "team2_node");
    ros::NodeHandle nh("/rrt_mpc");
    PurePursuit pp(nh);
    Predictor pred(nh,pp);
    std::cout << "predictor initialized" << std::endl;
    RRT rrt(nh,pred);
    std::cout << "rrt initialized" << std::endl;
    MPCManager mpc_manager(nh, rrt);
    std::cout << "mpc initialized" << std::endl;
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
    return 0;
}
