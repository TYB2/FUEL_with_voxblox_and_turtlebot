
#include "turtlebot_local_tracker_with_pid.h"

TurtlebotLocalTrackerWithPID::TurtlebotLocalTrackerWithPID(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private, tf2_ros::Buffer& tf)
    : nh_(nh), nh_private_(nh_private), tf_(tf), blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), controller_costmap_ros_(NULL){
    std::string local_planner;
    nh_private_.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    nh_private_.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_footprint"));

    nh_private_.param("controller_frequency", controller_frequency_, 20.0);
    nh_private_.param("controller_patience", controller_patience_, 15.0);

    nh_private_.param("oscillation_timeout", oscillation_timeout_, 0.0);
    nh_private_.param("oscillation_distance", oscillation_distance_, 0.5);

    nh_private_.param("shutdown_costmaps", shutdown_costmaps_, false);
    
    nh_private_.param("print_flag", print_flag, false);
    nh_private_.param("log_flag", log_flag, false);

    nh_.param("/robot/odom_type", odom_type, 1);


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
    initalizePIDController();

    sleep(1);
    // initAction();
}

TurtlebotLocalTrackerWithPID::~TurtlebotLocalTrackerWithPID(){
    tracker_.reset();
}

void TurtlebotLocalTrackerWithPID::init() {
    std::cout<<"TurtlebotLocalTrackerWithPID::init"<<std::endl;
  	tracker_server = nh_.advertiseService("/turtlebot_track", &TurtlebotLocalTrackerWithPID::trackerCallBack, this);
    clear_server = nh_.advertiseService("/turtlebot_stop", &TurtlebotLocalTrackerWithPID::clearCallBack, this);
    status_server = nh_.advertiseService("/turtlebot_status", &TurtlebotLocalTrackerWithPID::statusCallBack, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    vis_path_pub_ = nh_.advertise<visualization_msgs::Marker>("global_plan", 1000);
    // status_pub = nh_.advertise<std_msgs::Char>("tracker_status", 1000);
    status_pub = nh_.advertise<move_base::TrackerStatus>("tracker_status", 1);

    // odom_sub = nh_.subscribe("/odom", 10, &TurtlebotLocalTrackerWithPID::odomCallback, this);
    // odom_sub = nh_.subscribe("/gazebo/model_states", 10, &TurtlebotLocalTrackerWithPID::gazeboCallback, this);

    if(odom_type == 0){
        odom_sub = nh_.subscribe("/odom", 10, &TurtlebotLocalTrackerWithPID::odomCallback, this);
    }else if(odom_type == 1){
        odom_sub = nh_.subscribe("/gazebo/model_states", 10, &TurtlebotLocalTrackerWithPID::gazeboCallback, this);
    }

    odom_data_record_pub_ = nh_.advertise<move_base::OdomData>("/odom_data_record", 10);
}

void TurtlebotLocalTrackerWithPID::initalizePIDController(){
    maxAngularRate = M_PI;

    control_period = 0.1; 
    max_w = maxAngularRate; min_w = -maxAngularRate; 
  // Kp_w = 0.3; Ki_w = 0.05; Kd_w = 0.01;
    Kp_w = 1; Ki_w = 0.05; Kd_w = 0.01;

    pidTheta = PIDcontroller(control_period, max_w, min_w, Kp_w, Kd_w, Ki_w);
    angleTolerance=0.01;

    c_state = trackerState::Finished;
    track_id = 0;
    std::cout<<"\033[1;31m"<<"trackerState::Finished"<<"\033[0m"<<std::endl;
}

void TurtlebotLocalTrackerWithPID::odomCallback(const nav_msgs::Odometry odom_msg) {
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

void TurtlebotLocalTrackerWithPID::gazeboCallback(const gazebo_msgs::ModelStates odom_msg) {
    // std::cout<<"gazeboCallback "<<std::endl;

    current_pose.header.frame_id = "map";
    current_pose.header.stamp = ros::Time::now();
    current_pose.pose = odom_msg.pose.back();
    // std::cout<<current_pose.pose.position.x<<" "<<current_pose.pose.position.y<<std::endl;
}

bool TurtlebotLocalTrackerWithPID::clearCallBack(  move_base::ClearPath::Request& srv_req,
                                                    move_base::ClearPath::Response& srv_res){
track_id++;
c_state = trackerState::Finished;
publishZeroVelocity();
    //     controller_plan_ = srv_req.path;
    // new_global_plan_ = true;
    // std::cout<<"controller_plan_ "<<controller_plan_.size()<<std::endl;                                                                                                
}

void TurtlebotLocalTrackerWithPID::initAction(){

    geometry_msgs::PoseStamped pt;
    pt.header.frame_id = "map";
    pt.header.stamp = ros::Time::now();
    pt.pose.position.x = 0.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 0.0;

    for(double i = 0; i < 1; i += 0.1){
        pt.pose.position.x += 0.1;
        controller_plan_.push_back(pt);
    }
    
    pt.pose.orientation.z = sin( M_PI * 0.25);
    pt.pose.orientation.w = cos( M_PI * 0.25);
    controller_goal_ = pt;
    controller_turn_flag_ = true;
    new_global_plan_ = true;
    std::cout<<"controller_plan_ "<<controller_plan_.size()<<std::endl;

    if(controller_plan_.size() > 2){
        std::cout<<"begin to track"<<std::endl;
        c_state = trackerState::TurnS;
        std::cout<<"\033[1;31m"<<"trackerState::TurnS"<<"\033[0m"<<std::endl;
        last_oscillation_reset_ = ros::Time::now();
    }

    double turn_dist_thre = 0.01;
    for(int i = 0; i < controller_plan_.size(); i++){
        double diff_x = controller_plan_[i].pose.position.x - current_pose.pose.position.x;
        double diff_y = controller_plan_[i].pose.position.y - current_pose.pose.position.y;
        double dist_tmp = diff_x * diff_x + diff_y * diff_y;
        if(dist_tmp > turn_dist_thre){
            start_yaw = atan2(controller_plan_[i].pose.position.y - current_pose.pose.position.y, controller_plan_[i].pose.position.x - current_pose.pose.position.x);
            break;
        }
    }
    int n = controller_plan_.size() -1;
    end_yaw = atan(controller_goal_.pose.orientation.z/controller_goal_.pose.orientation.w) * 2;

}

bool TurtlebotLocalTrackerWithPID::statusCallBack(  move_base::TurtlebotStatus::Request& srv_req, 
                     move_base::TurtlebotStatus::Response& srv_res){

    if(c_state == trackerState::TurnS){
        srv_res.status = 'S';
    }else if(c_state == trackerState::Move){
        srv_res.status = 'M';
    }else if(c_state == trackerState::TurnE){
        srv_res.status = 'E';
    }else if(c_state == trackerState::Finished){
        srv_res.status = 'F';
    }
    
    return true;
}


bool TurtlebotLocalTrackerWithPID::trackerCallBack(  move_base::TrackerPath::Request& srv_req,
                                                    move_base::TrackerPath::Response& srv_res){
    controller_plan_ = srv_req.path;
    controller_goal_ = srv_req.goal;
    controller_turn_flag_ = srv_req.turn_flag;
    visualizePath(srv_req.path);
    new_global_plan_ = true;
    std::cout<<"controller_plan_ "<<controller_plan_.size()<<std::endl;

    if(controller_plan_.size() > 2){
        std::cout<<"begin to track"<<std::endl;
        c_state = trackerState::TurnS;
        std::cout<<"\033[1;31m"<<"trackerState::TurnS"<<"\033[0m"<<std::endl;
        last_oscillation_reset_ = ros::Time::now();
    }
    // start_yaw = atan2(controller_plan_[1].pose.position.y - controller_plan_[0].pose.position.y,
                                                        // controller_plan_[1].pose.position.x - controller_plan_[0].pose.position.x);
    // start_yaw = atan2(controller_plan_[0].pose.position.y - current_pose.pose.position.y,
                                                        // controller_plan_[0].pose.position.x - current_pose.pose.position.x);
    
    double turn_dist_thre = 0.01;
    for(int i = 0; i < controller_plan_.size(); i++){
        double diff_x = controller_plan_[i].pose.position.x - current_pose.pose.position.x;
        double diff_y = controller_plan_[i].pose.position.y - current_pose.pose.position.y;
        double dist_tmp = diff_x * diff_x + diff_y * diff_y;
        if(dist_tmp > turn_dist_thre){
            start_yaw = atan2(controller_plan_[i].pose.position.y - current_pose.pose.position.y, controller_plan_[i].pose.position.x - current_pose.pose.position.x);
            break;
        }
    }
    int n = controller_plan_.size() -1;
    end_yaw = atan(controller_goal_.pose.orientation.z/controller_goal_.pose.orientation.w) * 2;
    // ROS_WARN("get track!");
    // std::cout << "end_yaw" << end_yaw << std::endl;
}

  void TurtlebotLocalTrackerWithPID::publishZeroVelocity(){
    if(print_flag){
        std::cout<<"stop"<<std::endl;
    }
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  void TurtlebotLocalTrackerWithPID::visualizePath(std::vector<geometry_msgs::PoseStamped> &global_path){

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

  double TurtlebotLocalTrackerWithPID::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }


  void TurtlebotLocalTrackerWithPID::resetState(){
    // Disable the planner thread
    if(print_flag){
        std::cout<<"TurtlebotLocalTrackerWithPID::resetState"<<std::endl;
    }
    // Reset statemachine
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      controller_costmap_ros_->stop();
    }
  }

  bool TurtlebotLocalTrackerWithPID::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
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

bool TurtlebotLocalTrackerWithPID::track(){
    if(c_state == trackerState::TurnS){
        if(log_flag){
            ROS_INFO_STREAM("TurnS " << track_id);
        }
        turn(start_yaw);
    }else if(c_state == trackerState::Move){
        if(log_flag){
            ROS_INFO_STREAM("Move " << track_id);
        }
        move();
    }else if(c_state == trackerState::TurnE){
        if(log_flag){
            ROS_INFO_STREAM("TurnE " << track_id);
        }
        if(controller_turn_flag_){
            turn(end_yaw);
        }else{
            c_state = trackerState::Finished;
            track_id++;
            std::cout<<"\033[1;31m"<<"trackerState::Finished"<<"\033[0m"<<std::endl;
        }
    }else{
        if(log_flag){
            ROS_INFO_STREAM("Finished " << track_id);
        }
        publishZeroVelocity();
    }

    // std_msgs::Char status_msg;
    // if(c_state == trackerState::TurnS){
    //     status_msg.data = 'S';
    // }else if(c_state == trackerState::Move){
    //     status_msg.data = 'M';
    // }else if(c_state == trackerState::TurnE){
    //     status_msg.data = 'E';
    // }else if(c_state == trackerState::Finished){
    //     status_msg.data = 'F';
    // }

    move_base::OdomData tmp_odom_data; // record odom data
    move_base::TrackerStatus status_msg;
    if(c_state == trackerState::TurnS){
        status_msg.status = 'S';
        tmp_odom_data.status = 'S';// record odom data
    }else if(c_state == trackerState::Move){
        status_msg.status = 'M';
        tmp_odom_data.status = 'M';// record odom data
    }else if(c_state == trackerState::TurnE){
        status_msg.status = 'E';
        tmp_odom_data.status = 'E';// record odom data
    }else if(c_state == trackerState::Finished){
        status_msg.status = 'F';
        tmp_odom_data.status = 'F';// record odom data
    }
    status_msg.id = track_id;
    status_pub.publish(status_msg);

    // record odom data
    tmp_odom_data.cur_pose = current_pose.pose;
    tmp_odom_data.cur_time = (ros::Time::now()).toSec();
    odom_data_record_pub_.publish(tmp_odom_data);
}

bool TurtlebotLocalTrackerWithPID::turn(double target_yaw){
    double curr_yaw = 2.0 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w);

    // std::cout << "current yaw: " << curr_yaw << std::endl;
    // std::cout << "target yaw: " << target_yaw << std::endl;

    double diff_yaw = target_yaw - curr_yaw; 
    while(diff_yaw > M_PI){
        diff_yaw -= 2* M_PI;
    }
    while(diff_yaw <= -M_PI){
        diff_yaw += 2* M_PI;
    }

    if(abs(diff_yaw)<angleTolerance){
        if(print_flag){
            std::cout<<"Turned "<<cv::Point2d(curr_yaw,target_yaw)<<(target_yaw - curr_yaw)<<std::endl;
        }
        if(c_state == trackerState::TurnS){
            c_state = trackerState::Move;
            std::cout<<"\033[1;31m"<<"trackerState::Move"<<"\033[0m"<<std::endl;
            last_oscillation_reset_ = ros::Time::now();
        }
        if(c_state == trackerState::TurnE){
            c_state = trackerState::Finished;
            track_id++;
            std::cout<<"\033[1;31m"<<"trackerState::Finished"<<"\033[0m"<<std::endl;
        }
    }else{
        if(abs(curr_yaw - oscillation_yaw_) >= angleTolerance){
            last_oscillation_reset_ = ros::Time::now();
            oscillation_yaw_ = curr_yaw;
        }

        if(oscillation_timeout_ > 0.0 && last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()){
            std::cout<<"oscillation timeout reach"<<std::endl;
            publishZeroVelocity();
            if(c_state == trackerState::TurnS){
                c_state = trackerState::Move;
                std::cout<<"\033[1;31m"<<"trackerState::Move"<<"\033[0m"<<std::endl;
                last_oscillation_reset_ = ros::Time::now();
            }
            if(c_state == trackerState::TurnE){
                c_state = trackerState::Finished;
                track_id++;
                std::cout<<"\033[1;31m"<<"trackerState::Finished"<<"\033[0m"<<std::endl;
            }
            return true;
        }
        
        double omega = pidTheta.calculateU(diff_yaw,0);
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = omega;
        vel_pub_.publish(cmd_vel);
        
        if(print_flag){
            std::cout<<omega<<std::endl;
        }
    }
}

bool TurtlebotLocalTrackerWithPID::move(){
    if(print_flag){
        std::cout<<"TurtlebotLocalTrackerWithPID::move"<<std::endl;
    }
    
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
    }else{// if the change of the position is less then begin to timing...
        if(print_flag){
            std::cout<<"last_oscillation_reset_ + ros::Duration(oscillation_timeout_)  "
                                <<last_oscillation_reset_ + ros::Duration(oscillation_timeout_) <<"<"<< ros::Time::now()<<std::endl;
        }
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
            c_state = trackerState::TurnE;
            std::cout<<"\033[1;31m"<<"trackerState::TurnE"<<"\033[0m"<<std::endl;
            last_oscillation_reset_ = ros::Time::now();
            resetState();
            return true;
        }
    }

    //check for an oscillation condition
    if(oscillation_timeout_ > 0.0 && last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()){
        std::cout<<"oscillation timeout reach"<<std::endl;
        publishZeroVelocity();
        c_state = trackerState::TurnE;
        std::cout<<"\033[1;31m"<<"trackerState::TurnE"<<"\033[0m"<<std::endl;
        last_oscillation_reset_ = ros::Time::now();
        return true;
    }

    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        if(print_flag){
            std::cout<<"tracker_->computeVelocityCommands"<<std::endl;
        }
        if(tracker_->computeVelocityCommands(cmd_vel)){
            if(print_flag){
                std::cout<<"Got a valid command from the local planner: "<<cv::Point3d(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)<<std::endl;
            } 
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
                // std::cout<<"attempt_end"<<std::endl;
            }else{
                //otherwise, if we can't find a valid control, we'll go back to planning
                publishZeroVelocity();
                // std::cout<<"otherwise"<<std::endl;
            }
        }
    }
    //we aren't done yet
    return false;
}
