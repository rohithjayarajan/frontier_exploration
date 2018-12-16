/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    frontier_exploration_node.cpp
 *  @author  rohithjayarajan
 *  @date 16/2/2018
 *  @version 1.1
 *
 *  @brief source file for FrontierExplore class
 *
 *  @section DESCRIPTION
 *
 *  source file which contains the defintion of FrontierExplore
 * class
 *
 */
#include "frontier_exploration_node.hpp"
// C++ system header
#include <cstdint>
#include <utility>
#include <vector>
// BOOST header
#include <boost/range/irange.hpp>
// user defined header
#include "occupancy_map.hpp"
// ROS header
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

FrontierExplore::FrontierExplore() {
  ROS_INFO_STREAM("Turning on the turtlebot");
  // set frequency of publishing to 15Hz
  frequency_ = 15;
  // set velocity for angular motion(degrees/sec)
  angularVelZ_ = 0.628319;
  // set map to not loaded
  isMapLoaded = false;
  // initialize linear and angular motion values of the robot
  twistMsg_.linear.x = 0.0;
  twistMsg_.linear.y = 0.0;
  twistMsg_.linear.z = 0.0;
  twistMsg_.angular.x = 0.0;
  twistMsg_.angular.y = 0.0;
  twistMsg_.angular.z = 0.0;
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  vel_ = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",
                                            1000);
  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
  vel_.publish(twistMsg_);
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the
   * Subscriber object go out of scope, this callback will automatically be
   * unsubscribed from this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to
   * throw away the oldest ones.
   */
  map_ = nh.subscribe("/map", 1000, &FrontierExplore::updateMap, this);
  // set frame of goalPose
  goalPose.header.frame_id = "/map";
  // set orientation of goalPose
  goalPose.pose.orientation.w = 1;
}

FrontierExplore::~FrontierExplore() {
  // stop the linear and angular motion of
  // the robot
  twistMsg_.linear.x = 0.0;
  twistMsg_.linear.y = 0.0;
  twistMsg_.linear.z = 0.0;
  twistMsg_.angular.x = 0.0;
  twistMsg_.angular.y = 0.0;
  twistMsg_.angular.z = 0.0;
  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
  vel_.publish(twistMsg_);
}

void FrontierExplore::updateMap(
    const nav_msgs::OccupancyGrid::ConstPtr &grid_msg) {
  environmentMap.readOccupancyGrid(grid_msg);
}

bool FrontierExplore::updateRobotPose() {
  try {
    robotPose_.lookupTransform("/map", "/base_link", ros::Time(0), robotPose);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(3.0).sleep();
  }
  // return true on successful completion of function
  return true;
}

bool FrontierExplore::rotate() {
  ros::Rate loop_rate(frequency_);
  // stop the linear and angular motion of the robot
  twistMsg_.linear.x = 0.0;
  twistMsg_.linear.y = 0.0;
  twistMsg_.linear.z = 0.0;
  twistMsg_.angular.x = 0.0;
  twistMsg_.angular.y = 0.0;
  double rotVelocity = angularVelZ_;
  // set angular velocity
  twistMsg_.angular.z = rotVelocity;
  // time at instant before rotating
  ros::Time t0 = ros::Time::now();
  // time to stop rotation
  ros::Time t1 = t0 + ros::Duration(10.0);
  while (ros::Time::now() < t1 && ros::ok()) {
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    vel_.publish(twistMsg_);
    loop_rate.sleep();
  }
  // stop rotation
  twistMsg_.angular.z = 0;
  vel_.publish(twistMsg_);
  // return true on successful completion of function
  return true;
}

std::pair<double, double> FrontierExplore::findNearestFrontier(
    const std::vector<std::pair<double, double>> &frontierCentroidWorldSet) {
  // update the pose of the robot
  updateRobotPose();
  // get the x andy coordinate of the robot center
  auto robotXPos = robotPose.getOrigin().x();
  auto robotYPos = robotPose.getOrigin().y();
  ROS_INFO_STREAM("Pose of the robot is: x= " << robotXPos
                                              << " y= " << robotYPos);
  double frontierMinDist = std::numeric_limits<double>::max();
  double frontierDist;
  std::pair<double, double> nearestFrontier;
  // get the nearest frontier from all possible values of frontiers
  for (auto point : frontierCentroidWorldSet) {
    if (!(std::find(invalidPoints.begin(), invalidPoints.end(), point) !=
          invalidPoints.end())) {
      frontierDist =
          (std::hypot(robotXPos - point.first, robotYPos - point.second));
      // get frontier centroids that are more than 1.2 units away
      if (frontierDist < frontierMinDist && frontierDist > 1.2) {
        frontierMinDist = frontierDist;
        nearestFrontier = point;
      }
    }
  }
  // return the nearest frontier
  return nearestFrontier;
}

bool FrontierExplore::startExploration() {  // specifying rate at which to loop
  ros::Rate loop_rate(frequency_);
  while (ros::ok) {
    ROS_INFO_STREAM("Start rotation");
    bool rotateStat = rotate();
    ROS_INFO_STREAM("End rotatION");
    ros::spinOnce();
    // get frontier points and its count
    auto frontierPointsCOunt = environmentMap.getFrontiersPoints();
    ROS_INFO_STREAM(
        "The number of frontier points detected: " << frontierPointsCOunt);
    // get set of all frontier centroids in world coordinates
    auto frontierCentroidWorldSet = environmentMap.detectFrontiersCenterWorld();
    if (frontierCentroidWorldSet.size() != 0) {
      // get robot pose
      bool updateRobotPoseStat = updateRobotPose();
      // get nearest frontier
      auto nearestFrontier = findNearestFrontier(frontierCentroidWorldSet);
      ROS_INFO_STREAM("The nearest frontier detected is: x= "
                      << nearestFrontier.first
                      << "y = " << nearestFrontier.second);
      // create object of move base planner
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movePlanner(
          "move_base", true);
      // wait to connect to move base server
      while (!movePlanner.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      // create set point for the move base
      move_base_msgs::MoveBaseGoal setPoint;
      // set the specifics for the setPoint object
      setPoint.target_pose.header.frame_id = "/map";
      setPoint.target_pose.header.stamp = ros::Time::now();
      // set the goal for the robot
      setPoint.target_pose.pose.position.x = nearestFrontier.first;
      setPoint.target_pose.pose.position.y = nearestFrontier.second;
      setPoint.target_pose.pose.orientation.w = 1.0;
      // command the planner to go to the goal
      movePlanner.sendGoal(setPoint);
      // wait for response from move base planner
      movePlanner.waitForResult();
      if (movePlanner.getState() ==
          actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Successful navigation");
      } else {
        invalidPoints.push_back(nearestFrontier);
      }
    } else {
      ROS_INFO("Completed one iteration for the frontier exploration");
    }
  }
  // call the callbacks
  ros::spinOnce();
  // sleep for remaining time to hit 10Hz publish rate
  loop_rate.sleep();
  // return true on successful completion of function
  return true;
}
