

#include <vector>
#include <string>


#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <ros/ros.h>
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>

#include <pluginlib/class_loader.hpp>

#include <dynamic_reconfigure/server.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include "move_base/MoveBaseConfig.h"
#include <move_base/PlannerGoal.h>
#include <move_base/TrackerPath.h>

class TurtlebotGlobalPlanner{
    public:
    TurtlebotGlobalPlanner(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private, tf2_ros::Buffer& tf);
    ~TurtlebotGlobalPlanner();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber clicked_pose_sub;
    ros::Publisher path_pub;
    ros::Publisher goal_pub;
    ros::ServiceServer planner_server;
    ros::Subscriber odom_sub;

    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
    costmap_2d::Costmap2DROS* planner_costmap_ros_;
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

    std::vector<geometry_msgs::PoseStamped> global_plan;
    geometry_msgs::PoseStamped goal;

    bool goal_flag;
    double clicked_x;
    double clicked_y;
    double clicked_yaw;

    std::string robot_base_frame_, global_frame_;
    tf2_ros::Buffer& tf_;

    nav_msgs::Path path_msg;
    nav_msgs::Odometry goal_msg;

    void init();
    bool plan();
    void odomCallback(const nav_msgs::Odometry odom_msg);
    void clickedPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg);
    bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);
    bool trackerCallback(move_base::PlannerGoal::Request& req, move_base::PlannerGoal::Response& resp);
};
