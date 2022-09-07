#include <plan_env/voxblox_map.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

using namespace fast_planner;
using namespace std;

shared_ptr<SDFMap> sdf_map_;
visualization_msgs::Marker update_free_points;
visualization_msgs::Marker update_occupied_points;
visualization_msgs::Marker update_unknown_points;
Eigen::Vector4d free_color(1.0, 0, 0, 1.0);
Eigen::Vector4d occupied_color(0, 1.0, 0, 1.0);
Eigen::Vector4d unknown_color(0, 0, 1.0, 1.0);

void testCallback(const ros::TimerEvent& e){
    static int num = 0;

    ros::Time time_start = ros::Time::now();
    
    Eigen::Vector3d bmin(0, 0, 0);
    Eigen::Vector3d bmax(0, 0, 0);
    sdf_map_->getUpdatedBox(bmin, bmax, true);

    cout << num++ << " using time: " << (ros::Time::now() - time_start).toSec() << endl;
}

void test(){
    static int num = 0;

    ros::Time time_start = ros::Time::now();
    
    Eigen::Vector3d bmin(0, 0, 0);
    Eigen::Vector3d bmax(0, 0, 0);
    sdf_map_->getUpdatedBox(bmin, bmax, true);

    cout << num++ << " using time: " << (ros::Time::now() - time_start).toSec() << endl;
}

void fillBasicInfo(visualization_msgs::Marker& mk, const Eigen::Vector3d& scale,
                                          const Eigen::Vector4d& color, const string& ns, const int& id,
                                          const int& shape) {
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.ns = ns;
  mk.type = shape;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
}

void fillGeometryInfo(visualization_msgs::Marker& mk,
            const vector<Eigen::Vector3d>& list) {
  geometry_msgs::Point pt;

  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");

    sdf_map_.reset(new SDFMap(true, true));
    sdf_map_->initMap(nh);

    ros::Publisher updatedFreePointsShow = nh.advertise<visualization_msgs::Marker>("updatedPoints_free", 1000);
    ros::Publisher updatedOccupiedPointsShow = nh.advertise<visualization_msgs::Marker>("updatedPoints_occupied", 1000);
    ros::Publisher updatedUnknownPointsShow = nh.advertise<visualization_msgs::Marker>("updatedPoints_unknown", 1000);

    cout << "start test! "<< endl;

    // auto tmp_timer_ = nh.createTimer(ros::Duration(0.5), testCallback);

    double scale = 0.15;
    

    while(ros::ok()){
        std::vector<Eigen::Vector3d> allPoints;
        std::vector<Eigen::Vector3d> free_points_list;
        std::vector<Eigen::Vector3d> occupied_points_list;
        std::vector<Eigen::Vector3d> unknown_points_list;

        // test();
        allPoints = sdf_map_->getUpdatedPoint();
        cout << "size of allPoints" << allPoints.size() << endl;

        for(int i = 0; i < allPoints.size(); i++){
            auto state = sdf_map_->getOccupancy3D(allPoints[i]);
            if(state == SDFMap::FREE){
                free_points_list.push_back(allPoints[i]);
            }
            else if(state == SDFMap::OCCUPIED){
                occupied_points_list.push_back(allPoints[i]);
            }
            else{
                unknown_points_list.push_back(allPoints[i]);
            }
        }
        cout << "size of free_points_list" << free_points_list.size() << endl;
        cout << "size of occupied_points_list" << occupied_points_list.size() << endl;
        cout << "size of unknown_points_list" << unknown_points_list.size() << endl;
        
        /// free points
        fillBasicInfo(update_free_points, Eigen::Vector3d(scale, scale, scale), free_color,
                "updatePoints",  1, visualization_msgs::Marker::CUBE_LIST);
        update_free_points.action = visualization_msgs::Marker::ADD;
        fillGeometryInfo(update_free_points, free_points_list);
        updatedFreePointsShow.publish(update_free_points);
        update_free_points.points.clear();
        
        /// occupied points
        fillBasicInfo(update_occupied_points, Eigen::Vector3d(scale, scale, scale), occupied_color,
                "updatePoints",  1, visualization_msgs::Marker::CUBE_LIST);
        update_occupied_points.action = visualization_msgs::Marker::ADD;
        fillGeometryInfo(update_occupied_points, occupied_points_list);
        updatedOccupiedPointsShow.publish(update_occupied_points);
        update_occupied_points.points.clear();

        /// unkonwn points
        fillBasicInfo(update_unknown_points, Eigen::Vector3d(scale, scale, scale), unknown_color,
                "updatePoints",  1, visualization_msgs::Marker::CUBE_LIST);
        update_unknown_points.action = visualization_msgs::Marker::ADD;
        fillGeometryInfo(update_unknown_points, unknown_points_list);
        updatedUnknownPointsShow.publish(update_unknown_points);
        update_unknown_points.points.clear();

        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    // ros::spin();

    return 0;
}