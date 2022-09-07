#include "trajectory_recoder.h"

double floor_double(double src_num,double mod_num){
    double epsilon = 0.01;
    double res_num = floor((src_num + epsilon) / mod_num) * mod_num;
    return res_num;
}

TrajRecoder::TrajRecoder(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
    // manager_type = 1;
    init();
}

TrajRecoder::~TrajRecoder() {

}

void TrajRecoder::init() {
    std::cout<<"TrajRecoder::init"<<std::endl;
    nh_.param("/planner/manager_type", manager_type,1);
    nh_.param("/planner/odom_type", odom_type,0);

    voxel_size = 0.2;
    grid_size = 0.1;
    half_voxel_size = voxel_size / 2.0;
    odom_i = 0;
    grid_i = 0;

    visualizeInit();
    // odom_sub = nh_.subscribe("/odom", 10, &TrajRecoder::odomCallback, this);
    // odom_sub = nh_.subscribe("/gazebo/model_states", 10, &TrajRecoder::gazeboCallback, this);

    if(odom_type == 0){
        odom_sub = nh_.subscribe("/odom", 10, &TrajRecoder::odomCallback, this);
    }else if(odom_type == 1){
        odom_sub = nh_.subscribe("/gazebo/model_states", 10, &TrajRecoder::gazeboCallback, this);
    }
    
    vis_otraj_pub = nh_private_.advertise<visualization_msgs::MarkerArray>("odom_trajectory", 1000);
    vis_gtraj_pub = nh_private_.advertise<visualization_msgs::MarkerArray>("grid_trajectory", 1000);
    vis_opath_pub = nh_private_.advertise<nav_msgs::Path>("odom_path", 1000);
	traj_server = nh_.advertiseService("/get_trajectory", &TrajRecoder::trajectoryCallBack, this);
    timer = nh_.createTimer(ros::Duration(0.1), &TrajRecoder::timerCallback,this);

    odom_data_record_sub = nh_.subscribe("/odom_data_record", 10, &TrajRecoder::odomDataRecordCallback, this);
    save_to_file_trigger_sub = nh_.subscribe("/clicked_point", 1, &TrajRecoder::saveToFileTriggerCallback, this);
}

void TrajRecoder::odomCallback(const nav_msgs::Odometry &odom_msg) {
    // std::cout<<"odomCallback "<<cv::Point2d(odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)<<std::endl;

    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header.frame_id = "map";
    odom_pose.header.stamp = ros::Time::now();
    odom_pose.pose = odom_msg.pose.pose;
    geometry_msgs::PoseStamped grid_pose;
    grid_pose.header = odom_msg.header;
    grid_pose.pose = odom_msg.pose.pose;

    all_odom_path.poses.push_back(odom_pose);
    all_odom_id.push_back(-1);
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

    double grid_x = floor_double(curr_x - half_voxel_size, voxel_size)+ half_voxel_size;
    double grid_y = floor_double(curr_y - half_voxel_size, voxel_size)+ half_voxel_size;
    
    if(all_grid_path.poses.empty()){
        grid_pose.pose.position.x = grid_x;
        grid_pose.pose.position.y = grid_y;
        all_grid_path.poses.push_back(grid_pose);
        all_grid_id.push_back(-1);

    }else{
        geometry_msgs::PoseStamped prev_grid = all_grid_path.poses.back();
        if(grid_x != prev_grid.pose.position.x || grid_y != prev_grid.pose.position.y ){
            grid_pose.pose.position.x = grid_x;
            grid_pose.pose.position.y = grid_y;
            all_grid_path.poses.push_back(grid_pose);
            all_grid_id.push_back(-1);
        }
    }
    
    return;
}

void TrajRecoder::odomDataRecordCallback(const move_base::OdomData &odom_msg){
    OdomDataRecord cur_data;

    cur_data.cur_pose = odom_msg.cur_pose;
    cur_data.cur_time = odom_msg.cur_time;
    // cur_data.start_or_end_or_other = odom_msg.start_or_end_or_other;
    cur_data.status = odom_msg.status;

    odom_data_record_list.push_back(cur_data);
}

void TrajRecoder::saveToFileTriggerCallback(const geometry_msgs::PointStamped::ConstPtr& trigger_msg){
    static int flag = 0;

    if(trigger_msg->point.z < -0.1) return;

    if(flag == 0){
        flag++;
        return;
    }

    saveOdomDataToFile();
}

void TrajRecoder::saveOdomDataToFile(){

    // file name
    int cur_time = ros::Time::now().toSec();
    char tmp[10];
    sprintf(tmp, "%05d", cur_time);
    std::string file_name(tmp);
    file_name += "_robot_odom_data";
    std::string saved_file = odom_data_file_path + file_name;

    //save to file 
    std::ofstream file_out(saved_file, std::ios::out|std::ios::app|std::ios::binary);
    for(int i = 0; i < odom_data_record_list.size(); i++){
        file_out << odom_data_record_list[i].cur_time << std::endl;
        file_out << odom_data_record_list[i].status << std::endl;
        file_out << odom_data_record_list[i].cur_pose << std::endl;
        file_out << std::endl;
    }

    file_out.close();

    std::cout << "finish saving odom data" << std::endl;

}


void TrajRecoder::gazeboCallback(const gazebo_msgs::ModelStates odom_msg) {
    // std::cout<<"gazeboCallback"<<cv::Point2d(odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)<<std::endl;

    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header.frame_id = "map";
    odom_pose.header.stamp = ros::Time::now();
    odom_pose.pose = odom_msg.pose.back();
    geometry_msgs::PoseStamped grid_pose;
    grid_pose.header.frame_id = "map";
    grid_pose.header.stamp = ros::Time::now();
    grid_pose.pose = odom_msg.pose.back();

    all_odom_path.poses.push_back(odom_pose);
    all_odom_id.push_back(-1);
    double curr_x = odom_msg.pose.back().position.x;
    double curr_y = odom_msg.pose.back().position.y;
    double grid_x = floor_double(curr_x - half_voxel_size, voxel_size)+ half_voxel_size;
    double grid_y = floor_double(curr_y - half_voxel_size, voxel_size)+ half_voxel_size;
    
    if(all_grid_path.poses.empty()){
        grid_pose.pose.position.x = grid_x;
        grid_pose.pose.position.y = grid_y;
        all_grid_path.poses.push_back(grid_pose);
        all_grid_id.push_back(-1);
    }else{
        geometry_msgs::PoseStamped prev_grid = all_grid_path.poses.back();
        if(grid_x != prev_grid.pose.position.x || grid_y != prev_grid.pose.position.y ){
            grid_pose.pose.position.x = grid_x;
            grid_pose.pose.position.y = grid_y;
            all_grid_path.poses.push_back(grid_pose);
            all_grid_id.push_back(-1);
        }
    }

    return;
}

bool TrajRecoder::trajectoryCallBack(  move_base::GetTrajectory::Request& srv_req,
                                                                                move_base::GetTrajectory::Response& srv_res){
    last_time = srv_req.time;
    last_id = srv_req.id;

    std::cout<<"TrajRecoder::trajectoryCallBack"<<std::endl;
    std::cout<<"last_time "<<last_time <<std::endl;
    std::cout<<"last_id "<<last_id<<std::endl;
    std::cout<<"odom_i "<<odom_i<<std::endl;
    std::cout<<"grid_i "<<grid_i<<std::endl;

    std::cout<<"all_odom_path.poses.size() "<<all_odom_path.poses.size()<<std::endl;
    std::cout<<"all_grid_path.poses.size() "<<all_grid_path.poses.size()<<std::endl;

    stamps.push_back(last_time);

   while(odom_i < all_odom_path.poses.size()){
        if(all_odom_path.poses[odom_i].header.stamp <= last_time){
            all_odom_id[odom_i] = last_id;
            last_odom_pts.push_back(cv::Point2d(  all_odom_path.poses[odom_i].pose.position.x,
                                                                                                all_odom_path.poses[odom_i].pose.position.y));
        }else{
            break;
        }
        odom_i++;
    }
    std::cout<<"TrajRecoder::trajectoryCallBack 1"<<std::endl;

    while(grid_i < all_grid_path.poses.size()){
        if(all_grid_path.poses[grid_i].header.stamp <= last_time){
            all_grid_id[grid_i] = last_id;
            last_grid_pts.push_back(cv::Point2d(  all_grid_path.poses[grid_i].pose.position.x,
                                                                                            all_grid_path.poses[grid_i].pose.position.y));
            srv_res.traj.push_back(all_grid_path.poses[grid_i].pose.position);
        }else{
            break;
        }
        grid_i++;
    }
    std::cout<<"grid_i "<<grid_i<<std::endl;
    std::cout<<"srv_res.traj.size "<<srv_res.traj.size()<<std::endl;
    visualizeTrajectory();
    publish();
    return true;
}


void TrajRecoder::timerCallback(const ros::TimerEvent& e){
    // std::cout<<"TrajRecoder::timerCallback"<<std::endl;
    publish();
}

std_msgs::ColorRGBA pathColor(int traj_id){
    std_msgs::ColorRGBA color_tmp;
    double hue = (traj_id%12) / 12.0 * 360;
    double saturation = 1.0;
    double value = 1.0;

    double c = round(value * saturation);
    double h = hue / 60.0;
    double x = c * (1 - abs(fmod(h, 2.0) - 1));

    if ((h >= 0) && (h <= 1)){
        color_tmp.b = value - c;
        color_tmp.g = x + value - c;
        color_tmp.r = value;
    }else if ((h >= 1) && (h <= 2)){
        color_tmp.b = value - c;
        color_tmp.g = value;
        color_tmp.r = x + value - c;
    }else if ((h >= 2) && (h <= 3)){
        color_tmp.b = x + value - c;
        color_tmp.g = value;
        color_tmp.r = value - c;
    }else if ((h >= 3) && (h <= 4)){
        color_tmp.b = value;
        color_tmp.g = x + value - c;
        color_tmp.r = value - c;
    }else if ((h >= 4) && (h <= 5)){
        color_tmp.b = value;
        color_tmp.g = value - c;
        color_tmp.r = x + value - c;
    }else if ((h >= 5) && (h <= 6)){
        color_tmp.b = x + value - c;
        color_tmp.g = value - c;
        color_tmp.r = value;
    }
    color_tmp.a = 1.0;
    return color_tmp;
}

void TrajRecoder::visualizeInit(){
    std::cout<<"visualizeInit"<<std::endl;

    std_msgs::Header msg_header;
    msg_header.frame_id = "map";
    msg_header.stamp = ros::Time::now();

    all_odom_path.header = msg_header;
    int traj_id = odom_traj_marks_msg.markers.size();
    std_msgs::ColorRGBA color0 = pathColor(traj_id);

    visualization_msgs::Marker vis_traj;
    vis_traj.header = msg_header;
    vis_traj.id = 2 * traj_id;
    vis_traj.ns = "traj";
    vis_traj.type = visualization_msgs::Marker::LINE_LIST;
    vis_traj.action = visualization_msgs::Marker::ADD;
    vis_traj.pose.position.x = 0.0;
    vis_traj.pose.position.y = 0.0;
    vis_traj.pose.position.z = 0.0;
    vis_traj.pose.orientation.x = 0.0;
    vis_traj.pose.orientation.y = 0.0;
    vis_traj.pose.orientation.z = 0.0;
    vis_traj.pose.orientation.w = 1.0;
    vis_traj.scale.x = 0.02;
    vis_traj.color = color0;
    vis_traj.lifetime = ros::Duration(0.0);
    vis_traj.frame_locked = false;

    visualization_msgs::Marker vis_pt;
    vis_pt.header = msg_header;
    vis_pt.id = 2 * traj_id + 1;
    vis_pt.ns = "traj";
    vis_pt.type = visualization_msgs::Marker::CUBE_LIST;
    vis_pt.action = visualization_msgs::Marker::ADD;
    vis_pt.pose.position.x = 0.0;
    vis_pt.pose.position.y = 0.0;
    vis_pt.pose.position.z = 0.0;
    vis_pt.pose.orientation.x = 0.0;
    vis_pt.pose.orientation.y = 0.0;
    vis_pt.pose.orientation.z = 0.0;
    vis_pt.pose.orientation.w = 1.0;
    vis_pt.scale.x = 0.04;
    vis_pt.scale.y = 0.04;
    vis_pt.scale.z = 0.02;
    vis_pt.color = color0;
    vis_pt.lifetime = ros::Duration(0.0);
    vis_pt.frame_locked = false;

    odom_traj_marks_msg.markers.push_back(vis_traj);
    odom_traj_marks_msg.markers.push_back(vis_pt);

    grid_traj_marks_msg.markers.push_back(vis_traj);
    grid_traj_marks_msg.markers.push_back(vis_pt);

}

void TrajRecoder::visualizeTrajectory(){
    std::cout<<"visualizeTrajectory"<<std::endl;
    {
        visualization_msgs::Marker& vis_traj = odom_traj_marks_msg.markers[0];
        visualization_msgs::Marker& vis_pt = odom_traj_marks_msg.markers[1];
        vis_traj.points.clear();
        vis_pt.points.clear();

        for(int i = 1; i < last_odom_pts.size(); i++){
            geometry_msgs::Point point1;
            geometry_msgs::Point point2;
            point1.x = last_odom_pts[i - 1].x;
            point1.y = last_odom_pts[i - 1].y;
            point1.z = 0.0;
            point2.x = last_odom_pts[i].x;
            point2.y = last_odom_pts[i].y;
            point2.z = 0.0;
            vis_traj.points.push_back(point1);
            vis_traj.points.push_back(point2);
            vis_pt.points.push_back(point1);
        }
        last_odom_pts.clear();
    }

   {
        visualization_msgs::Marker& vis_traj = grid_traj_marks_msg.markers[0];
        visualization_msgs::Marker& vis_pt = grid_traj_marks_msg.markers[1];

        vis_traj.points.clear();
        vis_pt.points.clear();
        for(int i = 1; i < last_grid_pts.size(); i++){
            geometry_msgs::Point point1;
            geometry_msgs::Point point2;
            point1.x = last_grid_pts[i - 1].x;
            point1.y = last_grid_pts[i - 1].y;
            point1.z = 0.0;
            point2.x = last_grid_pts[i].x;
            point2.y = last_grid_pts[i].y;
            point2.z = 0.0;
            vis_traj.points.push_back(point1);
            vis_traj.points.push_back(point2);
            vis_pt.points.push_back(point1);
        }
        last_grid_pts.clear();
    }
}

void TrajRecoder::publish(){
    // vis_otraj_pub.publish(odom_traj_marks_msg);
    vis_gtraj_pub.publish(grid_traj_marks_msg);
    vis_opath_pub.publish(all_odom_path);
}