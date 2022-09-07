#ifndef PLAN_ENV_VOXBLOX_MAP_H
#define PLAN_ENV_VOXBLOX_MAP_H

#include <memory>

#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/ros_params.h>

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <queue>
#include <vector>
#include <ros/ros.h>
#include <tuple>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <voxblox_msgs/GetIncrementalVoxel.h>
#include <voxblox_msgs/UpdateResponse.h>

#include <nav_msgs/Odometry.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <random>

using namespace std;

namespace fast_planner{
    class SDFMap{
        public:
        SDFMap(bool save_map, bool sub_update_voxel);
        ~SDFMap();
        
        enum OCCUPANCY { UNKNOWN, FREE, OCCUPIED }; // 体素状态，UNKNOWN = 0，FREE = 1，OCCUPIED = 2

        typedef struct voxelStatusNum{
            int explored_voxel;
            int unexplored_voxel;
            int occupied_voxel;
            int free_voxel;
            double tsdf_rmse;
            
            voxelStatusNum(){};
            voxelStatusNum(int n1, int n2, int n3, int n4): explored_voxel(n1), unexplored_voxel(n2), occupied_voxel(n3), free_voxel(n4){ }
        }VoxelStatusNum;

        void initMap(ros::NodeHandle& nh, Eigen::Vector3d ThreeDimLen);
        void initMap(ros::NodeHandle& nh);

        // 需要替换的函数
        int getVoxelNum();
        double getResolution();
        void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
        bool getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset = false);
        void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
        void getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);
        void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
        bool isInBox(const Eigen::Vector3i& id);
        bool isInBox(const Eigen::Vector3d& pos);
        bool isInBox2D(const Eigen::Vector3i& id);
        bool isInBox2D(const Eigen::Vector3d& pos);
        int getInflateOccupancy(const Eigen::Vector3d& pos);
        int getInflateOccupancy(const Eigen::Vector3i& id);
        int getOccupancy(const Eigen::Vector3d& pos);
        int getOccupancy(const Eigen::Vector3i& id);
        int getOccupancy3D(const Eigen::Vector3d& pos);
        int getOccupancy3D(const Eigen::Vector3i& id);
        int toAddress(const Eigen::Vector3i& id);
        int toAddress(const int& x, const int& y, const int& z);
        void boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up);
        double getDistance(const Eigen::Vector3d& pos);
        double getDistance(const Eigen::Vector3i& id);
        double getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);

        // 额外的函数
        void calVoxelNum(Eigen::Vector3d ThreeDimLen);
        // void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
        void poseCallback(const nav_msgs::Odometry::ConstPtr& pose);
        void ESDFUpdateCallback(const voxblox_msgs::IncrementalVoxel &incremental_msg);
        std::vector<Eigen::Vector3d> getUpdatedPoint();
        void saveTsdfMap(string world_tsdf_file);
        void saveEsdfMap(string world_esdf_file);
        void saveMapCallback(const ros::TimerEvent& e);
        VoxelStatusNum evalVoxbloxMap(const string& world_esdf_file);

        protected:
        std::unique_ptr<voxblox::EsdfServer> esdf_server_;

        // 
        double collision_radius_;
        double c_voxel_size_;
        double c_block_size_;
        Eigen::Vector3d mapSize;
        Eigen::Vector3i map_voxel_num;

        // 这两个直接计算
        int VoxelNum;
        double Resolution;

        // 地图左下角和右上角坐标
        Eigen::Vector3d mapMinPos;
        Eigen::Vector3d mapMaxPos;

        //
        bool reset_updated_box_;
        Eigen::Vector3d update_min_;
        Eigen::Vector3d update_max_;
        Eigen::Vector3d map_ori_;

        //
        ros::Timer calBoundBox_timer_;
        ros::Timer saveMap_timer_;
        // std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber incremental_esdf_voxel_sub;

        ros::Publisher update_response_pub;

        //
        // geometry_msgs::PoseStamped curPose;
        nav_msgs::Odometry curPose;
        std::map<int, voxblox_msgs::IncrementalVoxel> map_incremental_msg;
        std::set<int> all_update_ids;

        // robot
        double cam_hight = 0.515;
        double robot_base = 0.2;
        double robot_height = 0.6;
        double floor_base = -0.1;

        // save map
        string world_tsdf_file_ = "/home/zjucvg/tmp_test/fuel_voxblox_turtlebot_ws/tsdf_files/";
        string world_esdf_file_ = "/home/zjucvg/tmp_test/fuel_voxblox_turtlebot_ws/esdf_files/";
        ros::Time start_time_;

        public:
        typedef std::shared_ptr<SDFMap> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    };
}

#endif