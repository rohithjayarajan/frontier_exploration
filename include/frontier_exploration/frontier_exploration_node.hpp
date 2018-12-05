/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    frontier_exploration_node.hpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief header file for FrontierExplore class
 *
 *  @section DESCRIPTION
 *
 *  header file which contains the declaration of FrontierExplore
 * class
 *
 */

#ifndef INCLUDE_FRONTIER_EXPLORATION_FRONTIER_EXPLORATION_NODE_HPP_
#define INCLUDE_FRONTIER_EXPLORATION_FRONTIER_EXPLORATION_NODE_HPP_
// C++ header
// #include <cmath>
// #include <sstream>
// #include <string>
// #include <boost/range/irange.hpp>
#include <utility>
#include <vector>
// ROS header
#include "geometry_msgs/Twist.h"
#include "map_structure.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

/**
 * @brief the declaration of FrontierExplore class
 *
 * Declaration of the variable and methods of FrontierExplore class
 */
class FrontierExplore {
 private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  // declare variable to hold message to be published
  geometry_msgs::Twist twistMsg_;
  // create subscriber for laserscan topic messages
  ros::Subscriber laserScan_;
  // create subscriber for MAP topic messages
  ros::Subscriber map_;
  // create subscriber for odom topic messages
  ros::Subscriber odom_;
  // create publisher to advertise messages to command motion of turtlebot
  ros::Publisher vel_;
  // declare variable to hold fixed value of angular velocity(degrees/sec)
  double angularVelZ_;
  // declare variable to hold distance threshold for a collision threat
  double minDist_;
  // declare variable to check if collision threat exists
  bool isCollision_;
  // declare variable which tells closest distance to potential collision
  double collisionDist_;
  // declare variable to set publishing rate
  double frequency_;
  // queue holding the frontier points
  std::vector<std::pair<int, int>> frontierCenter;
  // data structure containing the occupancy grid map
  std::vector<std::vector<MapStructure>> occupancyMap;
  // data structure for holding robot pose
  geometry_msgs::Pose robotPose;
  // data structure for holding goal pose
  geometry_msgs::PoseStamped goalPose;
  // variable to check if map is loaded
  bool isMapLoaded;

 public:
  /**
   *   @brief Default constructor for FrontierExplore
   *
   *   @param nothing
   *   @return nothing
   */
  FrontierExplore();
  /**
   *   @brief Default destructor for FrontierExplore
   *
   *   @param nothing
   *   @return nothing
   */
  ~FrontierExplore();
  /**
   *   @brief callback function for to monitor collision threat based on laser
   * scan messages
   *
   *   @param boost shared pointer to sensor_msgs::LaserScan
   *   @return nothing
   */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  /**
   *   @brief function to read the occupancy grid
   *
   *   @param nothing
   *   @return nothing
   */
  void readOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg);
  /**
   *   @brief function to check if the point is a frontier point
   *
   *   @param int values of index
   *   @return boolean value
   */
  bool isFrontier(const int &x_, const int &y_);
  /**
   *   @brief function to get frontier points
   *
   *   @param nothing
   *   @return nothing
   */
  void getFrontiersPoints();
  /**
   *   @brief function to group frontiers
   *
   *   @param nothing
   *   @return nothing
   */
  void classifyFrontiers();
  /**
   *   @brief function to get frontiers of the map
   *
   *   @param nothing
   *   @return nothing
   */
  void detectFrontiers();
  /**
   *   @brief function to get robot pose
   *
   *   @param boost shared pointer to nav_msgs::Odometry
   *   @return nothing
   */
  void updateRobotPose(const nav_msgs::Odometry::ConstPtr &odom_msg);
  /**
   *   @brief function to get goal pose from frontiers
   *
   *   @param nothing
   *   @return nothing
   */
  void updateGoalPose();
  /**
   *   @brief function to rotate the turtlebot by a value (in degrees) at a
   * fixed point
   *
   *   @param angle to rotate by (in degrees
   *   @return nothing
   */
  void rotate(const double &angleToRotate);
  /**
   *   @brief function to execute frontier exploaration task
   *
   *   @param nothing
   *   @return nothing
   */
  void startExploration();
};

#endif  // INCLUDE_FRONTIER_EXPLORATION_FRONTIER_EXPLORATION_NODE_HPP_
