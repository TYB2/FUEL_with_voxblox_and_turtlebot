#include "plan_env/voxblox_map.h"

namespace fast_planner{

    SDFMap::SDFMap(bool save_map, bool sub_update_voxel){
        ros::NodeHandle nh("");
        ros::NodeHandle nh_private("~");
        esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));

        collision_radius_ = 0.4;
        esdf_server_->setTraversabilityRadius(collision_radius_);

        c_voxel_size_ = esdf_server_->getEsdfMapPtr()->voxel_size();
        c_block_size_ = esdf_server_->getEsdfMapPtr()->block_size();

        reset_updated_box_ = true;
        update_min_ = update_max_ = Eigen::Vector3d(0, 0, 0);

        //
        map_ori_ << -10.0, -15.5, 0.0;

        // 设置定时器，计算UpdatedBox
        // calBoundBox_timer_ = nh.createTimer(ros::Duration(0.1), &SDFMap::calUpdatedBoxCallback, this);
    
        // get pose
        // pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/map_ros/pose", 25));
        // pose_sub_ = nh.subscribe("firefly/odometry_sensor1/odometry", 25, &SDFMap::poseCallback, this);
        pose_sub_ = nh.subscribe("/odom", 25, &SDFMap::poseCallback, this);

        if(sub_update_voxel){
            incremental_esdf_voxel_sub = nh.subscribe("/voxblox_node/esdf_incremental_pointcloud", 10, &SDFMap::ESDFUpdateCallback, this);
            update_response_pub = nh.advertise<voxblox_msgs::UpdateResponse>("/esdf_incremental_response", 100);
        }
        
        // save_map = false;
        if(save_map){
            saveMap_timer_ = nh.createTimer(ros::Duration(60), &SDFMap::saveMapCallback, this);
        }

        start_time_ = ros::Time::now();
    }

    SDFMap::~SDFMap(){
        esdf_server_.release();
    }

    void SDFMap::poseCallback(const nav_msgs::Odometry::ConstPtr& pose){
        curPose = *pose;
    }

    // 初始化
    void SDFMap::initMap(ros::NodeHandle& nh, Eigen::Vector3d ThreeDimLen){
        // 需要为VoxelNum和Resolution赋值；
        Resolution = c_voxel_size_;
        calVoxelNum(ThreeDimLen);

        // 赋值包围地图的长方体框的左下角和右上角坐标。这是需要根据地图的变化而变化的。
        mapMinPos << -10, -15.5, 0;
        mapMaxPos << 10, 4.5, 0.51; 
    } 

    // 初始化
    void SDFMap::initMap(ros::NodeHandle& nh){
        // 需要为VoxelNum和Resolution赋值；
        Resolution = c_voxel_size_;
        Eigen::Vector3d ThreeDimLen(20, 20, 0.51);
        calVoxelNum(ThreeDimLen);

        // 赋值包围地图的长方体框的左下角和右上角坐标。这是需要根据地图的变化而变化的。
        mapMinPos << -10, -15.5, 0;
        mapMaxPos << 10, 4.5, 0.51; 
    } 


    // 获取所有体素的总个数
    int SDFMap::getVoxelNum(){
        return VoxelNum;
    }

    // 获取体素分辨率
    double SDFMap::getResolution(){
        return Resolution;
    }

    // 获得地图原点坐标和xyz三个方向的尺寸  这里的尺寸和初始点还需要再修改一下
    void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size){
        ori = map_ori_;
        // ori << -10, -15.5, 0;  // 原点选择为（-10, -15.5, 0）
        // size << 20, 20, 10;  // 地图尺寸为20 20 10
        size = mapSize;
    }
    
    // 将BoundBox的相关信息输出
    bool SDFMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset){
        bmin = update_min_;
        bmax = update_max_;
        if (reset) reset_updated_box_ = true;
    }

    void SDFMap::ESDFUpdateCallback(const voxblox_msgs::IncrementalVoxel &incremental_msg){
        // if(update_data_flag){
        //     return;
        // }
        std::cout<<"VoxelmapManager::ESDFUpdateCallback"<<std::endl;

        voxblox_msgs::UpdateResponse response_msg;

        response_msg.update_ids.push_back(incremental_msg.update_id);
        update_response_pub.publish(response_msg);

        Eigen::Vector3d camera_pos;
        camera_pos.x() = curPose.pose.pose.position.x;
        camera_pos.y() = curPose.pose.pose.position.y;
        // camera_pos.z() = curPose.pose.pose.position.z;
        camera_pos.z() = 0.3;
        Eigen::Vector3d update_min = camera_pos;
        Eigen::Vector3d update_max = camera_pos;

        if(reset_updated_box_){
            update_min_ = camera_pos;
            update_max_ = camera_pos;
            reset_updated_box_ = false;
        }

        for(int i = 0; i < incremental_msg.points.size(); i++){
            if(incremental_msg.points[i].z > mapMaxPos.z()) continue;
            
            Eigen::Vector3d pt_w;
            pt_w << incremental_msg.points[i].x, incremental_msg.points[i].y, incremental_msg.points[i].z;

            for(int k = 0; k < 3; ++k){
                update_min[k] = std::min(update_min[k], pt_w[k]);
                update_max[k] = std::max(update_max[k], pt_w[k]);
            }
        }

        for (int k = 0; k < 3; ++k) {
            update_min_[k] = std::min(update_min[k], update_min_[k]);
            update_max_[k] = std::max(update_max[k], update_max_[k]);
        }
        
        // voxblox_msgs::UpdateResponse response_msg;

        // response_msg.update_ids.push_back(incremental_msg.update_id);
        // update_response_pub.publish(response_msg);

        // if(all_update_ids.find(incremental_msg.update_id) == all_update_ids.end() && 
        //     map_incremental_msg.find(incremental_msg.update_id) == map_incremental_msg.end() ){
        //     map_incremental_msg.insert(std::make_pair(incremental_msg.update_id,incremental_msg));
        // }
    }

    void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id){
        for(int i = 0; i < 3; i++){
            id[i] = (pos[i] - map_ori_[i]) / Resolution;
        }
    }

    void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos){
        for(int i = 0; i < 3; i++){
            pos[i] = id[i] * Resolution + map_ori_[i];
            pos[i] += Resolution / 2.0;
        }
    }

    // 得到地图左下角和右上角的坐标
    void SDFMap::getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax){
        bmin = mapMinPos;
        bmax = mapMaxPos;
    }

    // 根据输入的坐标，判断是否点是否在地图区域内 根据id
    bool SDFMap::isInBox(const Eigen::Vector3i& id){
        Eigen::Vector3d pos(0, 0, 0);
        indexToPos(id, pos);
        return isInBox(pos);
    }

    // 根据输入的坐标，判断是否点是否在地图区域内 根据pos
    bool SDFMap::isInBox(const Eigen::Vector3d& pos){

        for(int i = 0; i < 3; i++){
            if(pos[i] <= mapMinPos[i] || pos[i] >= mapMaxPos[i]) return false;
        }

        return true;
    }

        // 根据输入的坐标，判断是否点是否在地图区域内 根据id
    bool SDFMap::isInBox2D(const Eigen::Vector3i& id){
        Eigen::Vector3d pos(0, 0, 0);
        indexToPos(id, pos);
        return isInBox(pos);
    }

    // 根据输入的坐标，判断是否点是否在地图区域内 根据pos
    bool SDFMap::isInBox2D(const Eigen::Vector3d& pos){

        for(int i = 0; i < 2; i++){
            if(pos[i] <= mapMinPos[i] || pos[i] >= mapMaxPos[i]) return false;
        }

        return true;
    }

    // 暂时使用getOccupancy()代替该函数，即不膨胀 根据pos
    int SDFMap::getInflateOccupancy(const Eigen::Vector3d& pos){
        int state = getOccupancy(pos);
        if(state == UNKNOWN){
            return -1;
        }else if(state == OCCUPIED){
            return 1;
        }else if(state == FREE){
            return 0;
        }
    }

    // 暂时使用getOccupancy()代替该函数，即不膨胀 根据id
    int SDFMap::getInflateOccupancy(const Eigen::Vector3i& id){
        Eigen::Vector3d pos(0, 0, 0);
        indexToPos(id, pos);
        return getInflateOccupancy(pos);
    }

    // 获取当前体素的状态 根据pos
    int SDFMap::getOccupancy3D(const Eigen::Vector3d& pos){
        double dis = 0.0;
        if(esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(pos, &dis)){
            if(dis < c_voxel_size_){
                return OCCUPIED;
            }
            else return FREE;
        }
        else{
            return UNKNOWN;
        }
    }

    // 获取当前体素的状态 根据id
    int SDFMap::getOccupancy3D(const Eigen::Vector3i& id){
        Eigen::Vector3d pos(0, 0, 0);
        indexToPos(id, pos);
        return getOccupancy3D(pos);
    }

    int SDFMap::getOccupancy(const Eigen::Vector3d& pos){
        for(double zz = robot_base; zz <= robot_height; zz += Resolution) {
        // for(double zz = robot_base; zz <= robot_hight; zz += voxel_res) {
            Eigen::Vector3d point3(pos.x(), pos.y(), zz);
            int status_tmp = getOccupancy3D(point3);
            if (status_tmp == OCCUPIED) {
                return OCCUPIED;
            }
        }

        // for(double zz = floor_base; zz < robot_base; zz +=  grid_res) {
        double half_voxel_res = Resolution / 2.0;
        for(double zz = floor_base; zz < robot_base; zz += half_voxel_res) {
            Eigen::Vector3d point3(pos.x(), pos.y(), zz);
            int status_tmp = getOccupancy3D(point3);
            if (status_tmp == OCCUPIED) {
                return FREE;
            }
        }
        return UNKNOWN;
    }

    // 获取当前体素的状态 根据id
    int SDFMap::getOccupancy(const Eigen::Vector3i& id){
        Eigen::Vector3d pos(0, 0, 0);
        indexToPos(id, pos);
        return getOccupancy(pos);
    }

    // 这个函数的作用是把frontier_flag_和GlobalIndex一一对应起来  输入id
    int SDFMap::toAddress(const Eigen::Vector3i& id){
        return toAddress(id[0], id[1], id[2]);
    }

    // 这个函数的作用是把frontier_flag_和GlobalIndex一一对应起来  输入id的x, y, z
    int SDFMap::toAddress(const int& x, const int& y, const int& z){
        return x * map_voxel_num[1] * map_voxel_num[2] + y * map_voxel_num[2] + z;
    }

    // 直接抄了FUEL中的实现
    void SDFMap::boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up){
        for(int i = 0; i < 3; i++){
            low[i] = std::max(low[i], mapMinPos[i]);
            up[i] = std::min(up[i], mapMaxPos[i]);
        }
    }

    // 获取当前点距离最近障碍物的距离 根据pos
    double SDFMap::getDistance(const Eigen::Vector3d& pos){
        double dis = 0.0;
        esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(pos, &dis);

        return dis;
    }

    // 获取当前点距离最近障碍物的距离 根据id
    double SDFMap::getDistance(const Eigen::Vector3i& id){
        Eigen::Vector3d pos(0, 0, 0);
        indexToPos(id, pos);
        return getDistance(pos);
    }
    
    // 根据体积计算体素的数量
    void SDFMap::calVoxelNum(Eigen::Vector3d ThreeDimLen){
        mapSize = ThreeDimLen;
        map_voxel_num[0] = ThreeDimLen.x() / c_voxel_size_;
        map_voxel_num[1] = ThreeDimLen.y() / c_voxel_size_;
        map_voxel_num[2] = ThreeDimLen.z() / c_voxel_size_;
        VoxelNum = map_voxel_num[0] * map_voxel_num[1] * map_voxel_num[2];
    }

    // 
    double SDFMap::getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad){
        double dis;
        Eigen::Vector3d gradient;
        esdf_server_->getEsdfMapPtr()->getDistanceAndGradientAtPosition(pos, &dis, &gradient);

        grad = gradient;
        return dis;
    }


    std::vector<Eigen::Vector3d> SDFMap::getUpdatedPoint(){
        // std::vector<geometry_msgs::Point> incremental_voxels;
        std::vector<Eigen::Vector3d> incremental_voxels;
        for(std::map<int,voxblox_msgs::IncrementalVoxel>::iterator mit = map_incremental_msg.begin(); mit != map_incremental_msg.end(); mit++){
            voxblox_msgs::IncrementalVoxel incremental_voxel = mit->second;
            all_update_ids.insert(incremental_voxel.update_id);
            for(int i = 0; i < incremental_voxel.points.size(); i++){
                if(incremental_voxel.points[i].z > 0 || incremental_voxel.points[i].z < -0.2) continue;
                Eigen::Vector3d tmp(incremental_voxel.points[i].x, incremental_voxel.points[i].y, incremental_voxel.points[i].z);
                incremental_voxels.push_back(tmp);
            }
        }

        return incremental_voxels;
    }

    // save tsdf map
    void SDFMap::saveTsdfMap(string world_tsdf_file){
        static ros::Time start_time_local_ = ros::Time::now();
        int time_c_  = (int)((ros::Time::now() - start_time_local_).toSec());
        char tmp_s[10];
        sprintf(tmp_s, "%05d", time_c_);
        std::string tmp(tmp_s);
        world_tsdf_file += tmp;
        world_tsdf_file += ".tsdf";
        std::cout << "save to: " << world_tsdf_file << std::endl;
        if(esdf_server_->getTsdfMapPtr()->getTsdfLayerPtr()->saveToFile(world_tsdf_file)){
            std::cout << "succeed!"<< std::endl;
        }
        else{
            std::cout << "fail to save"<< std::endl;
        }
    }

    // useless
    void SDFMap::saveEsdfMap(string world_esdf_file){
        int time_c_  = (int)((ros::Time::now() - start_time_).toSec());
        world_esdf_file += to_string(time_c_);
        world_esdf_file += ".esdf";
        std::cout << "save to: " << world_esdf_file << std::endl;
        if(esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->saveToFile(world_esdf_file)){
            std::cout << "succeed!"<< std::endl;
        }
        else{
            std::cout << "fail to save"<< std::endl;
        }
    }


    void SDFMap::saveMapCallback(const ros::TimerEvent& e){
        static int flag = 0;

        int32_t duration_time = e.profile.last_duration.sec;

        cout << "flag: " << flag << endl;
        cout << "duration_time: " << duration_time << endl;
        if(flag <= 2){
            flag++;
            return;
        }
        saveTsdfMap(world_tsdf_file_);
        // saveEsdfMap(world_esdf_file_);
    }

    SDFMap::VoxelStatusNum SDFMap::evalVoxbloxMap(const string& world_tsdf_file){
        ros::NodeHandle nh("");
        ros::NodeHandle nh_private("~");

        std::unique_ptr<voxblox::TsdfServer> tsdf_world_server_;
        int explored_voxel, unexplored_voxel, occupied_voxel, free_voxel;

        occupied_voxel = free_voxel = 0;
        unexplored_voxel = VoxelNum;

        tsdf_world_server_.reset(new voxblox::EsdfServer(nh, nh_private));
        if(tsdf_world_server_->loadMap(world_tsdf_file)){
            std::cout << "succeed loading map" <<std::endl;
        }
        else{
            std::cout << "fail loading map" <<std::endl;
        }
        
        Eigen::Vector3i map_voxel_num_;
        map_voxel_num_[0] = (int)(20 / 0.2);
        map_voxel_num_[1] = (int)(20 / 0.2);
        map_voxel_num_[2] = (int)(0.51 / 0.2);
        unexplored_voxel = map_voxel_num_[0] * map_voxel_num_[1] * map_voxel_num_[2];
        std::cout << "size of map_voxel_num: " << map_voxel_num_.transpose() << std::endl;
        
        for(int x = 0; x < map_voxel_num_[0]; x++)
            for(int y = 0; y < map_voxel_num_[1]; y++)
                for(int z = 0; z < map_voxel_num_[2]; z++){
                    Eigen::Vector3d pos;
                    Eigen::Vector3i id(x, y, z);
                    indexToPos(id, pos);
                    voxblox::Point pt(pos[0], pos[1], pos[2]);

                    const voxblox::TsdfVoxel* voxel_cur_;
                    voxel_cur_ = tsdf_world_server_->getTsdfMapPtr()->getTsdfLayer().getVoxelPtrByCoordinates(pt);
                    
                    if(voxel_cur_ != nullptr){
                        if(voxel_cur_->distance < c_voxel_size_){
                            occupied_voxel++;
                        }
                        else{
                            free_voxel++;
                        }
                    }
                }
        
        explored_voxel = occupied_voxel + free_voxel;
        unexplored_voxel -= explored_voxel;

        SDFMap::VoxelStatusNum res(explored_voxel, unexplored_voxel, occupied_voxel, free_voxel);

        return res;
    }

    

    // position转GlobalVoxelIndex
    // void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id){
    //     // 根据pos得到block_id和voxel_id
    //     voxblox::VoxelIndex voxel_id;
    //     voxblox::BlockIndex block_id;
    //     Eigen::Vector3d center;
    //     block_id = esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->computeBlockIndexFromCoordinates(
    //             pos.cast<voxblox::FloatingPoint>());
    //     center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();

    //     // std::cout << "center: " << center << std::endl;
    //     // std::cout << "position: " << pos << std::endl;
    //     // std::cout << "c_voxel_size_: " << c_voxel_size_ << std::endl;

    //     voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
    //             (pos - center).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size_);  // why is "1.0/0.0" without error?

    //     // 根据block_id和voxel_id得到GlobalVoxelIndex
    //     size_t voxels_per_side_ = esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side();
    //     voxblox::GlobalIndex curGlobalIndex;
        
    //     // std::cout << "block_id: " << block_id << std::endl;
    //     // std::cout << "voxel_id: " << voxel_id << std::endl;
    //     // std::cout << "voxels_per_side_: " << voxels_per_side_ << std::endl;

    //     curGlobalIndex = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(block_id, voxel_id, voxels_per_side_);
    //     id << curGlobalIndex.x(), curGlobalIndex.y(), curGlobalIndex.z();
    // }

    // GlobalVoxelIndex转position  这个和上面那个pos转id的函数其实我也不太确定
    // void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos){
    //     // 输入GlobalVoxelIndex得到block_id和voxel_id
    //     size_t voxels_per_side_ = esdf_server_->getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side();
    //     voxblox::GlobalIndex curGlobalIndex;
    //     voxblox::VoxelIndex voxel_id;
    //     voxblox::BlockIndex block_id;

    //     curGlobalIndex << id.x(), id.y(), id.z();
    //     voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(curGlobalIndex, voxels_per_side_, &block_id, &voxel_id);
    //     // 根据block_id和voxel_id得到pos
    //     pos = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();
    //     pos += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_).cast<double>();
    // }
}