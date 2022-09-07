#include <eigen3/Eigen/Dense>
#include <ros/ros.h>


#include "trajectory_recoder.h"

geometry_msgs::PoseStamped next_goal;
bool goal_flag = false;
bool wait_flag = false;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_recode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    TrajRecoder traj_recoder(nh,nh_private);
    ros::spin();
    return 0;
}
