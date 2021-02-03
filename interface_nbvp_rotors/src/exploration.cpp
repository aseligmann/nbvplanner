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
#include <nbvplanner/nbvp_srv.h>

#include <std_srvs/SetBool.h>



// EVALUATING USING MAV_ACTIVE_3D_PLANNER EVALUATION FRAMEWORK //////////////////////////////
bool eval_planning_ = false;
bool eval_runSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    res.success = true;
    ROS_INFO_STREAM("EVAL: Got request for TOGGLE RUNNING.");
    if (req.data) {
        eval_planning_ = true;
        ROS_INFO("EVAL: Started planning.");
    } else {
        eval_planning_ = false;
        ROS_INFO("EVAL: Stopped planning.");
    }
    return true;
}
clock_t eval_cpu_srv_timer_ = std::clock();
bool eval_cpuSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    ROS_INFO_STREAM("EVAL: Got request for CPU service.");
    double time = (double) (std::clock() - eval_cpu_srv_timer_) / CLOCKS_PER_SEC;
    eval_cpu_srv_timer_ = std::clock();

    // Just return cpu time as the service message
    res.message = std::to_string(time).c_str();
    res.success = true;
    return true;
}
/////////////////////////////////////////////////////////////////////////////////////////////



int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
      > (mav_msgs::default_topics::COMMAND_TRAJECTORY, 5);
  ROS_INFO("Started exploration");
  std::string ns = ros::this_node::getName();

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  int gz_wait;
  nh.param(ns + "/gazebo_timeout", gz_wait, 10);
  while (i <= gz_wait && !unpaused) {
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

  double dt = 1.0;
  if (!ros::param::get(ns + "/nbvp/dt", dt)) {
    ROS_FATAL("Could not start exploration. Parameter missing! Looking for %s",
              (ns + "/nbvp/dt").c_str());
    return -1;
  }

  static int n_seq = 0;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();



  // EVALUATING USING MAV_ACTIVE_3D_PLANNER EVALUATION FRAMEWORK //////////////////////////////
  ros::ServiceServer eval_run_srv_ = nh.advertiseService("/planner_evaluation/toggle_running", eval_runSrvCallback);
  ros::ServiceServer eval_get_cpu_time_srv_ = nh.advertiseService("/planner_evaluation/get_cpu_time", eval_cpuSrvCallback);
  ros::Rate eval_r(1);
  while (!eval_planning_) {
    ROS_INFO("EVAL: Waiting for service request on %s to start planning...", eval_run_srv_.getService().c_str());
    ros::spinOnce();
    eval_r.sleep();
  }
  eval_cpu_srv_timer_ = std::clock();
  ROS_INFO("Continuing...");
  /////////////////////////////////////////////////////////////////////////////////////////////



  // This is the initialization motion, necessary that the known free space allows the planning
  // of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  for (double i = 0; i <= 1.0; i = i + 0.1) {
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
    ros::spinOnce();
  }
  trajectory_point.position_W.x() = 1.0;
  trajectory_point.position_W.y() = 0.0;
  samples_array.header.seq = n_seq;
  samples_array.header.stamp = ros::Time::now();
  samples_array.points.clear();
  n_seq++;
  mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  samples_array.points.push_back(trajectory_point_msg);
  trajectory_pub.publish(samples_array);
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 0;
  while (ros::ok() && eval_planning_) {
    ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
    nbvplanner::nbvp_srv planSrv;
    planSrv.request.header.stamp = ros::Time::now();
    planSrv.request.header.seq = iteration;
    planSrv.request.header.frame_id = "world";
    if (ros::service::call("nbvplanner", planSrv)) {
      n_seq++;
      if (planSrv.response.path.size() == 0) {
        ros::Duration(1.0).sleep();
      }
      for (int i = 0; i < planSrv.response.path.size(); i++) {
        samples_array.header.seq = n_seq;
        samples_array.header.stamp = ros::Time::now();
        samples_array.header.frame_id = "world";
        samples_array.points.clear();
        tf::Pose pose;
        tf::poseMsgToTF(planSrv.response.path[i], pose);
        double yaw = tf::getYaw(pose.getRotation());
        trajectory_point.position_W.x() = planSrv.response.path[i].position.x;
        trajectory_point.position_W.y() = planSrv.response.path[i].position.y;
        // Add offset to account for constant tracking error of controller
        trajectory_point.position_W.z() = planSrv.response.path[i].position.z + 0.25;
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
        trajectory_point.setFromYaw(tf::getYaw(quat));
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        trajectory_pub.publish(samples_array);
        ros::Duration(dt).sleep();
      }
    } else {
      ROS_WARN_THROTTLE(1, "Planner not reachable");
      ros::Duration(1.0).sleep();
    }
    iteration++;

    ros::spinOnce();
  }
}
