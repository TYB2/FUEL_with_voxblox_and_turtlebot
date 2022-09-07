
#include "turtlebot_local_tracker.h"

TurtlebotLocalTracker::TurtlebotLocalTracker(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private, tf2_ros::Buffer& tf)
    : nh_(nh), nh_private_(nh_private), tf_(tf), blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), controller_costmap_ros_(NULL){
    std::string local_planner;
    
    nh_private_.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    nh_private_.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_footprint"));

    nh_private_.param("controller_frequency", controller_frequency_, 20.0);
    nh_private_.param("controller_patience", controller_patience_, 15.0);

    nh_private_.param("oscillation_timeout", oscillation_timeout_, 0.0);
    nh_private_.param("oscillation_distance", oscillation_distance_, 0.5);

    nh_private_.param("shutdown_costmaps", shutdown_costmaps_, false);
    nh_.param("/robot/odom_type", odom_type, 0);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //initialize the global planner
    try {
      tracker_ = blp_loader_.createInstance(local_planner);
      std::cout<<"Created local_planner "<<local_planner<<std::endl;
      tracker_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }
    // Start actively updating costmaps based on sensor data
    controller_costmap_ros_->start();
    init();
}

TurtlebotLocalTracker::~TurtlebotLocalTracker(){
    tracker_.reset();
}

void TurtlebotLocalTracker::init() {
    std::cout<<"TurtlebotLocalTracker::init"<<std::endl;
  	tracker_server = nh_.advertiseService("/turtlebot_track", &TurtlebotLocalTracker::trackerCallBack, this);
    clear_server = nh_.advertiseService("/turtlebot_stop", &TurtlebotLocalTracker::clearCallBack, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    vis_path_pub_ = nh_.advertise<visualization_msgs::Marker>("global_plan", 1000);
    odom_sub = nh_.subscribe("/odom", 10, &TurtlebotLocalTracker::odomCallback, this);
}

void TurtlebotLocalTracker::odomCallback(const nav_msgs::Odometry odom_msg) {
    // std::cout<<"odomCallback "<<std::endl;
    double curr_x;
    double curr_y;
    double curr_yaw;

    double odom_x = odom_msg.pose.pose.position.x;
    double odom_y = odom_msg.pose.pose.position.y;
    double odom_yaw = 2.0 * atan2(odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w);
    
    if (!tf_listener.canTransform("/map", "/odom", odom_msg.header.stamp) || false){
        curr_x = odom_x;
        curr_y = odom_y;
        curr_yaw = odom_yaw;
    }else{
        tf::StampedTransform tf_transform;
        tf_listener.lookupTransform("/map", "/odom", odom_msg.header.stamp, tf_transform);

        double tf_x= tf_transform.getOrigin().getX();
        double tf_y=  tf_transform.getOrigin().getY();
        double tf_roll,tf_pitch,tf_yaw;
        tf::Matrix3x3(tf_transform.getRotation()).getEulerYPR(tf_yaw, tf_pitch, tf_roll);

        curr_x = cos(tf_yaw) * odom_x - sin(tf_yaw) * odom_y + tf_x;
        curr_y = sin(tf_yaw) * odom_x + cos(tf_yaw) * odom_y+ tf_y;
        curr_yaw = tf_yaw + odom_yaw;
    }

    // if (tf_listener.canTransform("/map", "/base_footprint", ros::Time::now()))
    // {
    //     tf::StampedTransform tf_transform;
    //     tf_listener.lookupTransform("/map", "/base_footprint", ros::Time::now(), tf_transform);
    //     // tf_listener.lookupTransform("/map", "/base_footprint", odom_msg.header.stamp, tf_transform);
    //     double tf_x= tf_transform.getOrigin().getX();
    //     double tf_y=  tf_transform.getOrigin().getY();
    //     double tf_roll,tf_pitch,tf_yaw;
    //     tf::Matrix3x3(tf_transform.getRotation()).getEulerYPR(tf_yaw, tf_pitch, tf_roll);
    //     curr_x = tf_x;
    //     curr_y =  tf_y;
    //     curr_yaw = tf_yaw;
    //     std::cout<<"odom_msg"<<odom_msg.pose.pose.position.x<<" "<<odom_msg.pose.pose.position.y<<"-"
    //                       <<"tf"<<curr_x<<" "<<curr_y<<std::endl;
    // // double odom_y = odom_msg.pose.pose.position.y;
    // }
    
    current_pose.header = odom_msg.header;
    current_pose.pose.position.x = curr_x;
    current_pose.pose.position.y = curr_y;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 0.0;
    current_pose.pose.orientation.z = sin(curr_yaw / 2.0);
    current_pose.pose.orientation.w = cos(curr_yaw / 2.0);
    // std::cout<<current_pose.pose.position.x<<" "<<current_pose.pose.position.y<<std::endl;
}

void TurtlebotLocalTracker::gazeboCallback(const gazebo_msgs::ModelStates odom_msg) {
    // std::cout<<"gazeboCallback "<<std::endl;

    current_pose.header.frame_id = "map";
    current_pose.header.stamp = ros::Time::now();
    current_pose.pose = odom_msg.pose.back();
    // std::cout<<current_pose.pose.position.x<<" "<<current_pose.pose.position.y<<std::endl;
}

bool TurtlebotLocalTracker::clearCallBack(  move_base::ClearPath::Request& srv_req,
                                                                                                move_base::ClearPath::Response& srv_res){
    //     controller_plan_ = srv_req.path;
    // new_global_plan_ = true;
    // std::cout<<"controller_plan_ "<<controller_plan_.size()<<std::endl;                                                                                                
}

bool TurtlebotLocalTracker::trackerCallBack(  move_base::TrackerPath::Request& srv_req,
                                                                                                move_base::TrackerPath::Response& srv_res){
    controller_plan_ = srv_req.path;
    visualizePath(srv_req.path);
    new_global_plan_ = true;
    last_oscillation_reset_ = ros::Time::now();
    std::cout<<"controller_plan_ "<<controller_plan_.size()<<std::endl;
}

  void TurtlebotLocalTracker::publishZeroVelocity(){
    std::cout<<"stop"<<std::endl;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  void TurtlebotLocalTracker::visualizePath(std::vector<geometry_msgs::PoseStamped> &global_path){

    std_msgs::Header msg_header;
    msg_header.frame_id = "map";
    msg_header.stamp = ros::Time::now();

    visualization_msgs::Marker vis_path;
    vis_path.header = msg_header;
    vis_path.id = 0;
    vis_path.ns = "path";
    vis_path.type = visualization_msgs::Marker::LINE_LIST;
    vis_path.action = visualization_msgs::Marker::ADD;
    vis_path.pose.position.x = 0.0;
    vis_path.pose.position.y = 0.0;
    vis_path.pose.position.z = 0.0;
    vis_path.pose.orientation.x = 0.0;
    vis_path.pose.orientation.y = 0.0;
    vis_path.pose.orientation.z = 0.0;
    vis_path.pose.orientation.w = 1.0;

    vis_path.scale.x = 0.02;
    vis_path.color.r = 1.0;
    vis_path.color.a = 1.0;
    vis_path.lifetime = ros::Duration(0.0);
    vis_path.frame_locked = false;

    for(int i = 0; i < global_path.size(); i++){
        geometry_msgs::Point point1;
        geometry_msgs::Point point2;
        point1.x = global_path[i].pose.position.x;
        point1.y = global_path[i].pose.position.y;
        double yaw_tmp = atan(global_path[i].pose.orientation.z/global_path[i].pose.orientation.w) * 2;
        point1.z = 0.0;
        point2.x = global_path[i].pose.position.x + 0.1 * cos(yaw_tmp);
        point2.y = global_path[i].pose.position.y + 0.1 * sin(yaw_tmp);
        vis_path.points.push_back(point1);
        vis_path.points.push_back(point2);
    }
    vis_path_pub_.publish(vis_path);
  }

  double TurtlebotLocalTracker::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }


  void TurtlebotLocalTracker::resetState(){
    // Disable the planner thread
    std::cout<<"TurtlebotLocalTracker::resetState"<<std::endl;
    // Reset statemachine
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      controller_costmap_ros_->stop();
    }
  }

  bool TurtlebotLocalTracker::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
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

bool TurtlebotLocalTracker::track(){
    std::cout<<"TurtlebotLocalTracker::track"<<std::endl;
    
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    // geometry_msgs::PoseStamped global_pose;
    // getRobotPose(global_pose, planner_costmap_ros_);
    // const geometry_msgs::PoseStamped& current_position = global_pose;

    //check to see if we've moved far enough to reset our oscillation timeout
    // if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    // {
    //   last_oscillation_reset_ = ros::Time::now();
    //   oscillation_pose_ = current_position;
    // }

    if(distance(current_pose, oscillation_pose_) >= oscillation_distance_){
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_pose;
    }else{
        std::cout<<"last_oscillation_reset_ + oscillation_timeout_  "
                            <<last_oscillation_reset_ + ros::Duration(oscillation_timeout_) <<"< now"<< ros::Time::now()<<std::endl;
        std::cout<<"oscillation_timeout_ "<<oscillation_timeout_<<std::endl;

    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent()){
        ROS_INFO("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
        publishZeroVelocity();
        return false;
    }

    //if we have a new plan then grab it and give it to the controller
    if(new_global_plan_){
        //make sure to set the new plan flag to false
        new_global_plan_ = false;

        ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

        std::cout<<"tracker_->setPlan "<<controller_plan_.size()<<std::endl;
        if(!tracker_->setPlan(controller_plan_)){
            //ABORT and SHUTDOWN COSTMAPS
            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
            resetState();
            return true;
        }

        //check to see if we've reached our goal
        if(tracker_->isGoalReached()){
            ROS_DEBUG_NAMED("move_base","Goal reached!");
            resetState();
            return true;
        }
    }

    //check for an oscillation condition
    if(oscillation_timeout_ > 0.0 && last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()){
        std::cout<<"oscillation timeout reach"<<std::endl;
        publishZeroVelocity();
        return true;
    }

    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        std::cout<<"tracker_->computeVelocityCommands"<<std::endl;
        if(tracker_->computeVelocityCommands(cmd_vel)){
            ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                            cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
            last_valid_control_ = ros::Time::now();
            //make sure that we send the velocity command to the base
            vel_pub_.publish(cmd_vel);

        }else {
            ROS_ERROR("The local planner could not find a valid plan.");
            ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
            
            //check if we've tried to find a valid control for longer than our time limit
            if(ros::Time::now() > attempt_end){
                //we'll move into our obstacle clearing mode
                publishZeroVelocity();
                std::cout<<"attempt_end"<<std::endl;

            }else{
                //otherwise, if we can't find a valid control, we'll go back to planning
                publishZeroVelocity();
                std::cout<<"otherwise"<<std::endl;
            }
        }
    }
    //we aren't done yet
    return false;
}