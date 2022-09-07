#include <move_base/move_base.h>
#include <tf2_ros/transform_listener.h>
#include "turtlebot_global_planner.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "Turtlebot_GlobalPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);


    TurtlebotGlobalPlanner global_planner(nh,nh_private,buffer);
    while (ros::ok()) {
        global_planner.plan();
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
     
    ros::spin();

    return 0;
}
