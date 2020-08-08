#include "predictor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "team2_mpc_node");
    ros::NodeHandle nh("/predictions");
    PurePursuit controller(nh);
    Predictor opponent_predictions(nh, controller);
    ros::spin();
    return 0;
}
