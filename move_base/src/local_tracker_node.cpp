#include <move_base/move_base.h>
#include <tf2_ros/transform_listener.h>
#include "turtlebot_local_tracker.h"
#include "turtlebot_local_tracker_with_pid.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "Turtlebot_GlobalPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);


    TurtlebotLocalTrackerWithPID local_tracker(nh,nh_private,buffer);
    // TurtlebotLocalTracker local_tracker(nh,nh_private,buffer);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        local_tracker.track();
        ros::spinOnce();
        // ros::Duration(1.0).sleep();
        loop_rate.sleep();
    }
     
    ros::spin();

    return 0;
}
