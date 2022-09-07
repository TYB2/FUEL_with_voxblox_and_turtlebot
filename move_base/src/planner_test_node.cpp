#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <move_base/PlannerGoal.h>

geometry_msgs::PoseStamped click_goal;
void clickedPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg){
    click_goal.header = pose_msg.header;
    click_goal.pose = pose_msg.pose.pose;

    move_base::PlannerGoal tracker_srv;
    tracker_srv.request.goal = pose_msg.pose.pose;
    ros::service::call("/tracker_goal", tracker_srv);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_recode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Subscriber clicked_pose_sub = nh.subscribe("/initialpose", 10,clickedPoseCallback);
    ros::spin();
    return 0;
}
