
#ifndef TURTLEBOT_LOCAL_TRACKER_WITH_PID_H
#define TURTLEBOT_LOCAL_TRACKER_WITH_PID_H

#include <vector>
#include <string>

#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
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
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/ModelStates.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>
#include <std_msgs/Char.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"
#include <move_base/TrackerPath.h>
#include <move_base/ClearPath.h>
#include <move_base/TrackerStatus.h>
#include <move_base/TurtlebotStatus.h>

// #include "common.h"
#include "pid_controller.h"

#include <move_base/OdomData.h>


enum trackerState {
    TurnS = 0,
    Move = 1,
    TurnE = 2,
    Finished =3
};


class TurtlebotLocalTrackerWithPID{
    public:
    TurtlebotLocalTrackerWithPID(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private, tf2_ros::Buffer& tf);
    ~TurtlebotLocalTrackerWithPID();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::ServiceServer tracker_server;
    ros::ServiceServer clear_server;
    ros::ServiceServer status_server;
    ros::Publisher vel_pub_;
    ros::Publisher odom_data_record_pub_;
    ros::Publisher vis_path_pub_;
    ros::Subscriber odom_sub;
    ros::Publisher status_pub;
    tf::TransformListener tf_listener;

    int track_id;
    bool print_flag;
    bool log_flag;

    boost::shared_ptr<nav_core::BaseLocalPlanner> tracker_;
    costmap_2d::Costmap2DROS* controller_costmap_ros_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;

    std::vector<geometry_msgs::PoseStamped> controller_plan_;
    geometry_msgs::PoseStamped controller_goal_;
    bool controller_turn_flag_;
    geometry_msgs::PoseStamped goal;

    geometry_msgs::PoseStamped current_pose;
    visualization_msgs::MarkerArray vis_path;

    ros::Time last_valid_control_, last_oscillation_reset_;
    geometry_msgs::PoseStamped oscillation_pose_;
    double oscillation_timeout_, oscillation_distance_;
    double oscillation_yaw_;
    double last_plan_reset_;
    double plan_timeout_;

    double  controller_frequency_;
    double  controller_patience_;

    bool shutdown_costmaps_;

    bool new_global_plan_;

    std::string robot_base_frame_, global_frame_;
    tf2_ros::Buffer& tf_;

    trackerState c_state;
    double start_yaw;
    double end_yaw;

    PIDcontroller pidTheta; 
    double maxAngularRate;
    double angleTolerance;
    double control_period;//It Should be consistent with rate of ros
    double max_w;
    double min_w;
    double Kp_w;
    double Ki_w;
    double Kd_w;
    int odom_type;
    void init();
    void initalizePIDController();
    bool track();
    bool move();
    bool turn(double target_yaw);

    void odomCallback(const nav_msgs::Odometry odom_msg);
    void gazeboCallback(const gazebo_msgs::ModelStates odom_msg);

    bool trackerCallBack(  move_base::TrackerPath::Request& srv_req, move_base::TrackerPath::Response& srv_res);
    bool statusCallBack(  move_base::TurtlebotStatus::Request& srv_req, move_base::TurtlebotStatus::Response& srv_res);
    bool clearCallBack(  move_base::ClearPath::Request& srv_req, move_base::ClearPath::Response& srv_res);
    void visualizePath(std::vector<geometry_msgs::PoseStamped> &global_path);

    void resetState();
    void publishZeroVelocity();
    bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);
    double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

    // test
    void initAction();
};

#endif
