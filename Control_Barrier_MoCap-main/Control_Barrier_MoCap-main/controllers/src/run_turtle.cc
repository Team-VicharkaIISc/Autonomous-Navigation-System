/*
 * simulate.cc
 *
 *  created: Oct 2016
 *   author: Matthias Rungger
 */

/*
 * information about this example is given in
 * http://arxiv.org/abs/1503.03715
 * doi: 10.1109/TAC.2016.2593947
 */

#include <iostream>
#include <array>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <phasespace_msgs/Markers.h>

#include <barrier_controller/Robot.h>
#include <barrier_controller/Robots.h>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"

/* state space dim */
const int state_dim = 3;
/* input space dim */
const int input_dim = 2;

/* sampling time */
const double tau = 2.1;

/*
 * data types for the state space elements and input space
 * elements used in uniform grid and ode solvers
 */
using state_type = std::array<double, state_dim>;
using input_type = std::array<double, input_dim>;

geometry_msgs::Pose2D curr_pose_1;
geometry_msgs::Pose2D curr_pose_2;

void tbPoseCallback_1(const phasespace_msgs::Markers &msg) {
  phasespace_msgs::Marker marker_dyn = msg.markers[0];
  phasespace_msgs::Marker marker_ori = msg.markers[1];

  curr_pose_1.x = marker_dyn.x;
  curr_pose_1.y = marker_dyn.y;

  double dy = marker_dyn.y - marker_ori.y;
  double dx = marker_dyn.x - marker_ori.x;

  double angle = std::atan2(dy, dx);
  curr_pose_1.theta = angle;
}

void tbPoseCallback_2(const phasespace_msgs::Markers &msg) {
  phasespace_msgs::Marker marker_dyn = msg.markers[3];
  phasespace_msgs::Marker marker_ori = msg.markers[2];

  curr_pose_2.x = marker_dyn.x;
  curr_pose_2.y = marker_dyn.y;

  double dy = marker_dyn.y - marker_ori.y;
  double dx = marker_dyn.x - marker_ori.x;

  double angle = std::atan2(dy, dx);
  curr_pose_2.theta = angle;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "run_turtlebot");
  
  ros::NodeHandle nh;
  
  ros::Subscriber marker_sub = nh.subscribe("/phasespace/markers", 0, tbPoseCallback_1);
  ros::Subscriber marker_sub_2 = nh.subscribe("/phasespace/markers", 0, tbPoseCallback_2);

  ros::Publisher vel_robots = nh.advertise<barrier_controller::Robots>("/robots", 0);

  /* define function to check if we are in target */
  auto target_1 = [](const state_type& x) {
    if (2.5 <= x[0] && x[0] <= 2.85 && 0.65 <= x[1] && x[1] <= 1.0)
      return true;
    return false;
  };

  /* define function to check if we are in target */
  auto target_2 = [](const state_type& x) {
    if (2.5 <= x[0] && x[0] <= 2.85 && 3.8 <= x[1] && x[1] <= 4.15)
      return true;
    return false;
  };

  /* read controller from file */
  scots::StaticController con_1;
  if(!read_from_file(con_1, "controller_tb_1")) {
    std::cout << "Could not able to read from controller_tb_2.scs\n";
    return 0;
  }

  /* read controller from file */
  scots::StaticController con_2;
  if(!read_from_file(con_2, "controller_tb_2")) {
    std::cout << "Could not able to read from controller_tb_2.scs\n";
    return 0;
  }

  // state_type x = {curr_pose.x, curr_pose.y, curr_pose.theta};

  std::cout << "\nRobot Started:\n " << std::endl;

  while(ros::ok()) {

    ros::spinOnce();

    state_type x_1 = {curr_pose_1.x, curr_pose_1.y, curr_pose_1.theta};
    state_type x_2 = {curr_pose_2.x, curr_pose_2.y, curr_pose_2.theta};

    barrier_controller::Robot tb_1;
    barrier_controller::Robot tb_2;

    if(!target_1(x_1)) {
      std::cout << "Curr Robot Pose 1: " << x_1[0] <<  " "  << x_1[1] << " " << x_1[2] << "\n";

      std::vector<input_type> u_1 = con_1.peek_control<state_type, input_type>(x_1);

      tb_1.id = 0;
      tb_1.flag = false;
      tb_1.u.push_back(u_1[0][0]);
      tb_1.u.push_back(u_1[0][1]);
    }
    else {
      tb_1.id = 0;
      tb_1.flag = true;
      tb_1.u.push_back(0.0);
      tb_1.u.push_back(0.0);
    }

    if(!target_2(x_2)) {
      std::cout << "Curr Robot Pose 2: " << x_2[0] <<  " "  << x_2[1] << " " << x_2[2] << "\n";

      std::vector<input_type> u_2 = con_2.peek_control<state_type, input_type>(x_2);

      tb_2.id = 1;
      tb_2.flag = false;
      tb_2.u.push_back(u_2[0][0]);
      tb_2.u.push_back(u_2[0][1]);
    }
    else {
      tb_2.id = 1;
      tb_2.flag = true;
      tb_2.u.push_back(0.0);
      tb_2.u.push_back(0.0);
    }

    barrier_controller::Robots tb3s;

    tb3s.robots.push_back(tb_1);
    tb3s.robots.push_back(tb_2);

    ros::Time beginTime = ros::Time::now();
    ros::Duration secondsIWantToSendMessagesFor = ros::Duration(tau); 
    ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;
    
    while(ros::Time::now() < endTime )
    {
        // cmd_vel_pub.publish(vel);
        vel_robots.publish(tb3s);

        // Time between messages, so you don't blast out an thousands of 
        // messages in your 3 secondperiod
        ros::Duration(0.1).sleep();
    }
  }

  return 0;
}
