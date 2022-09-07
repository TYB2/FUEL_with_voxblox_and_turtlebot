#include "turtlebot_global_planner.h"

TurtlebotGlobalPlanner::TurtlebotGlobalPlanner(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private, tf2_ros::Buffer& tf)
    : nh_(nh), nh_private_(nh_private), tf_(tf), bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), planner_costmap_ros_(NULL){
    std::string global_planner;
    nh_private_.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    nh_private_.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_footprint"));
    nh_private_.param("global_costmap/global_frame", global_frame_, std::string("map"));
    
    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    init();
}

TurtlebotGlobalPlanner::~TurtlebotGlobalPlanner(){
    planner_.reset();
}

void TurtlebotGlobalPlanner::init() {
    std::cout<<"TurtlebotGlobalPlanner::init"<<std::endl;
    clicked_pose_sub = nh_.subscribe("/xinitialpose", 10,&TurtlebotGlobalPlanner::clickedPoseCallback, this);
    path_pub = nh_private_.advertise<nav_msgs::Path>("global_path", 1000);
    goal_pub = nh_private_.advertise<nav_msgs::Odometry>("global_goal", 1000);
    planner_server = nh_.advertiseService("tracker_goal", &TurtlebotGlobalPlanner::trackerCallback, this);
    odom_sub = nh_.subscribe("/odom", 10, &TurtlebotGlobalPlanner::odomCallback, this);
}

void TurtlebotGlobalPlanner::odomCallback(const nav_msgs::Odometry odom_msg) {

}


void TurtlebotGlobalPlanner::clickedPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_msg){
    // planner_costmap_ros_->stop();

    goal_flag = true;
    clicked_x = pose_msg.pose.pose.position.x;
    clicked_y = pose_msg.pose.pose.position.y;
    clicked_yaw = 2.0 * atan2(pose_msg.pose.pose.orientation.z,pose_msg.pose.pose.orientation.w);

    goal.header = pose_msg.header;
    goal.pose = pose_msg.pose.pose;
    path_msg.header = pose_msg.header;
    goal_msg.header = pose_msg.header;
    return;
}

  bool TurtlebotGlobalPlanner::trackerCallback(move_base::PlannerGoal::Request& req, move_base::PlannerGoal::Response& resp){
    std::cout<<"begin to plan"<<std::endl;
    // planner_costmap_ros_->stop();
    goal_flag = true;
    
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose = req.goal;
    path_msg.header = goal.header;
    goal_msg.header = goal.header;
    return true;
  }

  bool TurtlebotGlobalPlanner::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (!global_pose.header.stamp.isZero() &&
        current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }
    return true;
}

bool TurtlebotGlobalPlanner::plan(){
    if(!goal_flag){
        return false;
    }
    goal_flag = false;

    // // debug
    // ROS_WARN("start planning!");
    // double end_yaw_debug = atan(goal.pose.orientation.z/goal.pose.orientation.w) * 2;
    // std::cout << "goal yaw: " << end_yaw_debug << std::endl;

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));
    global_plan.clear();
    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
        ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
        return false;
    }

    //get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
        ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
        return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, global_plan) || global_plan.empty()){
        ROS_ERROR_STREAM( "Failed to find a  plan to point: " <<"("<<goal.pose.position.x<<","<< goal.pose.position.y<<")" );
        return false;
    }

    std::cout<<"move base has found  a valid plan !"<<std::endl;
    std::cout<<"planner_->makePlan "<<global_plan.size()<<std::endl;

    path_msg.poses = global_plan;
    goal_msg.pose.pose = goal.pose;
    path_pub.publish(path_msg);
    goal_pub.publish(goal_msg);

    move_base::TrackerPath tracker_path_srv;
    tracker_path_srv.request.path = global_plan;
    tracker_path_srv.request.goal = goal;
    tracker_path_srv.request.turn_flag = true;
    ros::service::call("/turtlebot_track", tracker_path_srv);

    // // debug
    // ROS_WARN("end planning!");
    // end_yaw_debug = atan(goal.pose.orientation.z/goal.pose.orientation.w) * 2;
    // std::cout << "goal yaw: " << end_yaw_debug << std::endl;
    
    return true;
}
