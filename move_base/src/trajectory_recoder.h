#ifndef TRAJECTORY_RECODER_H_
#define TRAJECTORY_RECODER_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base/GetTrajectory.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

#include <move_base/OdomData.h>

#include <fstream>

double floor_double(double src_num,double mod_num);

std_msgs::ColorRGBA pathColor(int traj_id);

typedef struct odomDataRecord{
    char status;
    // char start_or_end_or_other;
    geometry_msgs::Pose cur_pose;
    double cur_time;
}OdomDataRecord;

class TrajRecoder {
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber odom_sub;
    ros::Subscriber odom_data_record_sub;
    ros::Subscriber path_sub;
    ros::Subscriber save_to_file_trigger_sub;
    ros::ServiceServer traj_server;
    ros::Publisher vis_otraj_pub;
    ros::Publisher vis_gtraj_pub;
    ros::Publisher vis_opath_pub;
    tf::TransformListener tf_listener;

    ros::Timer timer;

    std::vector<OdomDataRecord> odom_data_record_list;
    
    TrajRecoder(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~TrajRecoder();
    void init();
    void odomCallback(const nav_msgs::Odometry &odom_msg);
    void gazeboCallback(const gazebo_msgs::ModelStates odom_msg);
    void odomDataRecordCallback(const move_base::OdomData &odom_msg);
    void saveToFileTriggerCallback(const geometry_msgs::PointStamped::ConstPtr& trigger_msg);

    void pathCallback(const nav_msgs::Path &path_msg);
    void timerCallback(const ros::TimerEvent& e);
    bool trajectoryCallBack(  move_base::GetTrajectory::Request& srv_req,
                                                        move_base::GetTrajectory::Response& srv_res);
    void publish();             
    void visualizeInit();
    void visualizeTrajectory();
    void saveOdomDataToFile();
    int manager_type;
    int odom_type;
    
    nav_msgs::Path all_odom_path;
    std::vector<int> all_odom_id;
    nav_msgs::Path all_grid_path;
    std::vector<int> all_grid_id;

    int grid_i;
    int odom_i;

    std::vector<ros::Time> stamps;
    ros::Time last_time;
    int last_id;
    std::vector<cv::Point2d> last_odom_pts;
    std::vector<cv::Point2d> last_grid_pts;

    double voxel_size;
    double grid_size;
    double half_voxel_size;

    visualization_msgs::MarkerArray odom_traj_marks_msg;
    visualization_msgs::MarkerArray grid_traj_marks_msg;

    //
    std::string odom_data_file_path{"/home/zjucvg/tmp_test/fuel_voxblox_turtlebot_ws/odom_data/"};
};

#endif // TRAJECTORY_RECODER_H_
