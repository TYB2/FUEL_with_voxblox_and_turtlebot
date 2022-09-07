
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
// #include <plan_env/sdf_map.h>
#include <plan_env/voxblox_map.h>

#include <time.h>

using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "INIT_ACTION", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  /* Ros sub, pub and timer */
  timerIsOn_ = false;
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);  // 执行动作命令 
  // safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this); // 检查是否会发生碰撞
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this); // 边界点检测

  // trigger_sub_ =
  //     nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
  trigger_sub_ = nh.subscribe("/clicked_point", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
  tracker_status_sub_ = nh.subscribe("/tracker_status", 1, &FastExplorationFSM::trackerStatusCallback, this);

  // replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  // new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  // bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10); // 


}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  static int tmp = 0;
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

  static time_t time_last = 0; 
  time_t time_cur;
  time(&time_cur);
  if(time_last == 0){
    std::cout << "time_last init" << std::endl;
    time_last = time_cur;
  }
  else if((time_cur - time_last) > 60){
    std::cout << "start saving tsdf" << std::endl;
    time_last = time_cur;
    std::string path("/home/zjucvg/tmp_test/fuel_voxblox_turtlebot_ws/tsdf_files/");
    expl_manager_->planner_manager_->edt_environment_->sdf_map_->saveTsdfMap(path);
    std::cout << "finish saving tsdf" << std::endl;
  }

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(INIT_ACTION, "FSM");
      break;
    }

    case INIT_ACTION: {
      initAction();
      // ros::service::call("/turtlebot_track", tracker_path_srv);
      if(!ros::service::call("/tracker_goal", tracker_srv)){
        ROS_ERROR("fail calling /tracker_goal in INIT_ACTION");
      }

      // // debug
      // double yaw_debug = atan(tracker_srv.request.goal.orientation.z / tracker_srv.request.goal.orientation.w) * 2.0;
      // std::cout << "input yaw: " << yaw_debug << std::endl;

      transitState(WAIT_TRIGGER, "FSM");

      time(&globle_start_time);
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      time_t time_current_;
      time(&time_current_);
      std::cout << "running time: " << (time_current_ - globle_start_time) << std::endl;
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      // if (fd_->static_state_) {
      //   // Plan from static state (hover)
      //   fd_->start_pt_ = fd_->odom_pos_;
      //   fd_->start_vel_ = fd_->odom_vel_;
      //   fd_->start_acc_.setZero();

      //   fd_->start_yaw_(0) = fd_->odom_yaw_;
      //   fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      // } else {
      //   // Replan from non-static state, starting from 'replan_time' seconds later
      //   LocalTrajData* info = &planner_manager_->local_data_;
      //   double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

      //   // 这是在做什么
      //   fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
      //   fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
      //   fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
      //   fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
      //   fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
      //   fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      // }

      {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      }

      // Inform traj_server the replanning
      // replan_pub_.publish(std_msgs::Empty());
      // code here. stop moving
      // clear_path_srv.request.stop = true;
      // if(!ros::service::call("/turtlebot_stop", clear_path_srv)){
      //   ROS_ERROR("fail calling /turtlebot_stop in PLAN_TRAJ");
      // }

      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        transitState(FINISH, "FSM");
        ROS_WARN("all finished");
        fd_->static_state_ = true;
        clearVisMarker();
      } else if (res == FAIL) {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("plan fail");
        fd_->static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        // bspline_pub_.publish(fd_->newest_traj_);
        // ros::service::call("/turtlebot_track", tracker_path_srv);
        if(!ros::service::call("/tracker_goal", tracker_srv)){
          ROS_ERROR("fail calling /tracker_goal in PUB_TRAJ");
        }
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      // double time_to_end = info->duration_ - t_cur;
      // if (time_to_end < fp_->replan_thresh1_) {
      //   transitState(PLAN_TRAJ, "FSM");
      //   ROS_WARN("Replan: traj fully executed=================================");
      //   return;
      // }
      if(!ros::service::call("/turtlebot_status", turtlebot_srv)){
          ROS_ERROR("fail calling /turtlebot_status in EXEC_TRAJ");
      }
      if (turtlebot_srv.response.status == 'F' && t_cur > fp_->replan_thresh2_) {  // get robot state
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      // if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
      //   transitState(PLAN_TRAJ, "FSM");
      //   ROS_WARN("Replan: cluster covered=====================================");
      //   return;
      // }
      // Replan after some time
      if (t_cur > fp_->replan_thresh3_ && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: periodic call=======================================");
      }
      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ROS_WARN("call exploration planner");

  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_); // replan_time is a const value !
  Eigen::Vector3d nextGoal;
  double nextYaw;

  int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_, nextGoal, nextYaw);
  // int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
  //                                            fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);
  if (res != SUCCEED) return res;

  // {
  //   auto info = &planner_manager_->local_data_;
  //   info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

  //   bspline::Bspline bspline;
  //   bspline.order = planner_manager_->pp_.bspline_degree_;
  //   bspline.start_time = info->start_time_;
  //   bspline.traj_id = info->traj_id_;
  //   Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  //   for (int i = 0; i < pos_pts.rows(); ++i) {
  //     geometry_msgs::Point pt;
  //     pt.x = pos_pts(i, 0);
  //     pt.y = pos_pts(i, 1);
  //     pt.z = pos_pts(i, 2);
  //     bspline.pos_pts.push_back(pt);
  //   }
  //   Eigen::VectorXd knots = info->position_traj_.getKnot();
  //   for (int i = 0; i < knots.rows(); ++i) {
  //     bspline.knots.push_back(knots(i));
  //   }
  //   Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  //   for (int i = 0; i < yaw_pts.rows(); ++i) {
  //     double yaw = yaw_pts(i, 0);
  //     bspline.yaw_pts.push_back(yaw);
  //   }
  //   bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
  //   fd_->newest_traj_ = bspline;
  // }

  // {
  //   auto info = &planner_manager_->local_data_;
  //   info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
  //   tracker_path_srv.request.path.clear();

  //   Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  //   double goal_yaw = yaw_pts(yaw_pts.rows()-1, 0);
  //   tracker_path_srv.request.goal.header.frame_id = "map";
  //   tracker_path_srv.request.goal.header.stamp = ros::Time::now();

  //   Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  //   geometry_msgs::PoseStamped pt;
  //   pt.header.frame_id = "map";
  //   std::cout<<"pos_pts: ";
  //   for (int i = 0; i < pos_pts.rows(); i++) {
  //     pt.header.stamp = ros::Time::now();
  //     pt.pose.position.x = pos_pts(i, 0);
  //     pt.pose.position.y = pos_pts(i, 1);
  //     tracker_path_srv.request.path.push_back(pt);
  //     std::cout<<cv::Point2d(pos_pts(i, 0),pos_pts(i, 1));
  //   }
  //   std::cout<<std::endl;
  //   pt.pose.orientation.z = sin(goal_yaw / 2.0);
  //   pt.pose.orientation.w = cos(goal_yaw / 2.0);
  //   tracker_path_srv.request.goal = pt;
  //   tracker_path_srv.request.turn_flag = true;

  //   tracker_srv.request.goal = pt.pose;
  // }

  {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;
    
    geometry_msgs::PoseStamped pt;
    pt.header.frame_id = "map";
    pt.header.stamp = ros::Time::now();
    pt.pose.position.x = nextGoal[0];
    pt.pose.position.y = nextGoal[1];
    pt.pose.orientation.z = sin(nextYaw / 2.0);
    pt.pose.orientation.w = cos(nextYaw / 2.0);

    tracker_srv.request.goal = pt.pose;

    std::cout << cv::Point(pt.pose.position.x, pt.pose.position.y) << std::endl;
  }

  return res;
}

void FastExplorationFSM::visualize() {
  // auto info = &planner_manager_->local_data_;
  // auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  cout << "start visualize" << endl;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.6),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  
  Eigen::Vector3d cur_goal_(tracker_srv.request.goal.position.x, tracker_srv.request.goal.position.y, 0.1);
  visualization_->drawGoal(cur_goal_, 0.1, Vector4d(1, 0, 0, 1), 6);

  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  // Draw local refined viewpoints info
  // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
  //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  // 1),
  //                           "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  // 0, 1),
  //                           "refine_pair", 0, 6);
  // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
  //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
  //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
  //                               ed_ptr->frontiers_.size()),
  //                               "n_points", i, 6);
  // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
  //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // Draw trajectory
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  // visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
  //                             Vector4d(1, 1, 0, 1));
  // std::vector<Eigen::Vector3d> list; 
  // std::cout<<"cmd path :";
  // for(int i = 0; i <tracker_path_srv.request.path.size();i++){
  //   Eigen::Vector3d pt;
  //   pt[0] = tracker_path_srv.request.path[i].pose.position.x;
  //   pt[1] = tracker_path_srv.request.path[i].pose.position.y;
  //   pt[2] = tracker_path_srv.request.path[i].pose.position.z;
  //   list.push_back(pt);
  //   std::cout<<cv::Point3d(pt[0],pt[1],pt[2]);
  // }
  // std::cout<<std::endl;
  // visualization_->displayLineList2(list, 
  //                                 0.1, Eigen::Vector4d(0, 1, 0, 1), 2, 2);
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  cout << "start frontierCallback and delay: " << delay << endl;
  if (++delay < 5) return;

  // don't repeat operating
  if (timerIsOn_ == true) return;
  timerIsOn_ = true;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {  // 如果再等待或者已执行完上次路径
    cout << "start frontierCallback 1" << endl;
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_; // exploration data
    
    cout << "searchFrontiers" << endl;

    ft->searchFrontiers();

//     ft->getDebugFrontiers(ed->frontiers_);
// {
//   std::cout << "size of ed frontiers: " << ed->frontiers_.size() << std::endl;
//   for (auto it = ed->frontiers_.begin(); it != ed->frontiers_.end(); ++it) {
//     std::cout<<(*it).size()<<" ";
//   }
//   std::cout<<std::endl;
// }

//     for (int i = 0; i < ed->frontiers_.size(); ++i) {
//       visualization_->drawCubes(ed->frontiers_[i], 0.1,
//                                 visualization_->getColor(double(i) / ed->frontiers_.size(), 0.6),
//                                 "frontier", i, 4);
//     }

//     ft->computeFrontiersToVisit();

//     vector<vector<Eigen::Vector3d>> tmp_sample_points_;

//     ft->getDebugSamplePoints(tmp_sample_points_);

//     for (int i = 0; i < tmp_sample_points_.size(); ++i) {
//       if(tmp_sample_points_[i].empty()) continue;
//       Eigen::Vector4d color = visualization_->getColor(double(i) / tmp_sample_points_.size(), 0.6);
//       cout << " curent color: " << color.transpose() << endl;
//       visualization_->drawSpheres(tmp_sample_points_[i], 0.1,
//                                 color,
//                                 "frontier", i, 6);
//     }

    // ft->updateFrontierCostMatrix();

    // ft->getDebugFrontiers(ed->frontiers_);
    // ft->getFrontiers(ed->frontiers_);

    // for (int i = 0; i < ed->frontiers_.size(); ++i) {
    //   visualization_->drawCubes(ed->frontiers_[i], 0.1,
    //                             Vector4d(0.5, 0, 1, 1.0),
    //                             "frontier", i, 4);
    // }

    // return;

    cout << "computeFrontiersToVisit" << endl;

    ft->computeFrontiersToVisit();

    cout << "updateFrontierCostMatrix" << endl;

    ft->updateFrontierCostMatrix();

    cout << "getFrontiers" << endl;

    ft->getFrontiers(ed->frontiers_);

    cout << "getFrontierBoxes" << endl;

    ft->getFrontierBoxes(ed->frontier_boxes_);

    // Draw frontier and bounding box
    cout << "size of frontiers_(after filter)" << ed->frontiers_.size() << endl;
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.15,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.6),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    // for (int i = ed->frontiers_.size(); i < 50; ++i) {
    //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    //   // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    //   // "frontier_boxes", i, 4);
    // }
  }

  timerIsOn_ = false;

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = ros::Time::now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (ros::Time::now() - t1).toSec();
  //   ROS_WARN("Average astar time: %lf", astar_time);
  // }
}

void FastExplorationFSM::triggerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  // if (msg->poses[0].pose.position.z < -0.1) return;
  if (msg->point.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
       << endl;
}

void FastExplorationFSM::trackerStatusCallback(const move_base::TrackerStatus::ConstPtr& msg) {
  tracker_status = msg->status;
  tracker_id = msg->id;
}

void FastExplorationFSM::initAction() {

  // tracker_path_srv.request.goal.header.frame_id = "map";
  // tracker_path_srv.request.goal.header.stamp = ros::Time::now();

  // Eigen::Vector3d cur_pos(0, 0, 0);

  // geometry_msgs::PoseStamped pt;
  // pt.header.frame_id = "map";
  // pt.header.stamp = ros::Time::now();
  // for (double i = 0; i < 1.5; i += 0.1) {
  //   cur_pos.x() += 0.1;
  //   pt.pose.position.x = cur_pos.x();
  //   pt.pose.position.y = cur_pos.y();
  //   pt.pose.position.z = 0.0;
  //   tracker_path_srv.request.path.push_back(pt);
  // }
  // tracker_path_srv.request.goal = pt;

  // double goal_yaw = M_PI * 0.5;
  // tracker_path_srv.request.goal.pose.orientation.z = sin(goal_yaw / 2.0);
  // tracker_path_srv.request.goal.pose.orientation.w = cos(goal_yaw / 2.0);
  // tracker_path_srv.request.turn_flag = true;


  geometry_msgs::PoseStamped pt;
  pt.header.frame_id = "map";
  pt.header.stamp = ros::Time::now();
  pt.pose.position.x = 1.6;
  pt.pose.position.y = 0;
  double goal_yaw = M_PI / 2.0;
  pt.pose.orientation.z = sin(goal_yaw / 2.0);
  pt.pose.orientation.w = cos(goal_yaw / 2.0);
  tracker_srv.request.goal = pt.pose;
}

}  // namespace fast_planner
