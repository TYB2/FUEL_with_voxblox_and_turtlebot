/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
      > (mav_msgs::default_topics::COMMAND_TRAJECTORY, 5);
  ROS_INFO("Started exploration");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  static int n_seq = 0;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  // This is the initialization motion, necessary that the known free space allows the planning
  // of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  for (double i = 0; i <= 2.5; i = i + 0.1) {
    nh.param<double>("wp_x", trajectory_point.position_W.x(), 0.0);
    nh.param<double>("wp_y", trajectory_point.position_W.y(), 0.0);
    nh.param<double>("wp_z", trajectory_point.position_W.z(), 1.0);
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -M_PI * i);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(1.0).sleep();
  }

  // for(double i = 0; i <= 2.0; i += 0.1){
  //   trajectory_point.position_W.y() -= 0.1;
  //   samples_array.header.seq = n_seq;
  //   samples_array.header.stamp = ros::Time::now();
  //   samples_array.points.clear();
  //   n_seq++;
  //   mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  //   samples_array.points.push_back(trajectory_point_msg);
  //   trajectory_pub.publish(samples_array);
  //   ros::Duration(1.0).sleep();
  // }

    trajectory_point.position_W.y() -= 3.0;
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(1.0).sleep();


  for (double i = 0.5; i <= 1.6; i = i + 0.1) {
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -M_PI * i);
    trajectory_point.position_W.x() += 0.0;
    trajectory_point.position_W.y() += 0.0;
    trajectory_point.position_W.z() += 0.0;
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(1.0).sleep();
  }

  trajectory_point.position_W.y() += 3.0;
  samples_array.header.seq = n_seq;
  samples_array.header.stamp = ros::Time::now();
  samples_array.points.clear();
  n_seq++;
  mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  samples_array.points.push_back(trajectory_point_msg);
  trajectory_pub.publish(samples_array);
  ros::Duration(1.0).sleep();

  for (double i = 1.6; i <= 2.6; i = i + 0.1) {
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();
    n_seq++;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -M_PI * i);
    trajectory_point.position_W.x() += 0.0;
    trajectory_point.position_W.y() += 0.0;
    trajectory_point.position_W.z() += 0.0;
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    trajectory_pub.publish(samples_array);
    ros::Duration(1.0).sleep();
  }

  // trajectory_point.position_W.x() -= 0.5;
  // trajectory_point.position_W.y() -= 0.5;
  // samples_array.header.seq = n_seq;
  // samples_array.header.stamp = ros::Time::now();
  // samples_array.points.clear();
  // n_seq++;
  // mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  // samples_array.points.push_back(trajectory_point_msg);
  // trajectory_pub.publish(samples_array);
  // ros::Duration(1.0).sleep();

  // trajectory_point.position_W.x() += 0.5;
  // trajectory_point.position_W.y() += 0.5;
  // samples_array.header.seq = n_seq;
  // samples_array.header.stamp = ros::Time::now();
  // samples_array.points.clear();
  // n_seq++;
  // mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  // samples_array.points.push_back(trajectory_point_msg);
  // trajectory_pub.publish(samples_array);
  // ros::Duration(1.0).sleep();

  // trajectory_point.position_W.x() += 1.0;
  // trajectory_point.position_W.y() += 0.5;
  // samples_array.header.seq = n_seq;
  // samples_array.header.stamp = ros::Time::now();
  // samples_array.points.clear();
  // n_seq++;
  // mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  // samples_array.points.push_back(trajectory_point_msg);
  // trajectory_pub.publish(samples_array);
  // ros::Duration(1.0).sleep();

  // trajectory_point.position_W.x() -= 1.0;
  // trajectory_point.position_W.y() -= 2.0;
  // samples_array.header.seq = n_seq;
  // samples_array.header.stamp = ros::Time::now();
  // samples_array.points.clear();
  // n_seq++;
  // mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  // samples_array.points.push_back(trajectory_point_msg);
  // trajectory_pub.publish(samples_array);
  // ros::Duration(1.0).sleep();

  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 0;
  while (ros::ok()) {
    // ... need service call
    iteration++;
  }
}
