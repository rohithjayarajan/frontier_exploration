/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    frontier_exploration_node.hpp
 *  @author  rohithjayarajan
 *  @date 14/2/2018
 *  @version 1.1
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
#include <cstdint>
#include <utility>
#include <vector>
#include "occupancy_map.hpp"
// ROS header
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/range/irange.hpp>
#include "geometry_msgs/Twist.h"
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
  // create subscriber for MAP topic messages
  ros::Subscriber map_;
  // create subscriber for odom topic messages
  ros::Subscriber odom_;
  // create publisher to advertise messages to command motion of turtlebot
  ros::Publisher vel_;
  // declare variable to hold fixed value of angular velocity(degrees/sec)
  double angularVelZ_;
  // declare variable which tells closest distance to potential collision
  double collisionDist_;
  // declare variable to set publishing rate
  double frequency_;
  // queue holding the frontier points
  std::vector<std::pair<int, int>> frontierCenter;
  // listener for robot pose
  tf::TransformListener robotPose_;
  // data structure for holding robot pose
  tf::StampedTransform robotPose;
  // data structure for holding goal pose
  geometry_msgs::PoseStamped goalPose;
  // occupancy grid map of the environment
  OccupancyMap environmentMap;
  // variable to check if map is loaded
  bool isMapLoaded;
  // list of points not allowed to go to
  std::vector<std::pair<double, double>> invalidPoints;

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
   *   @brief callback function for to update occupancy map based on map
   * messages
   *
   *   @param boost shared pointer to nav_msgs::OccupancyGrid
   *   @return boolean true on successful completion
   */
  bool updateMap(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg);
  /**
   *   @brief function to get robot pose
   *
   *   @param nothing
   *   @return boolean true on successful completion
   */
  bool updateRobotPose();
  /**
   *   @brief function to get goal pose from frontiers
   *
   *   @param nothing
   *   @return boolean true on successful completion
   */
  bool updateGoalPose();
  /**
   *   @brief function to rotate the turtlebot by a value (in degrees) at a
   * fixed point
   *
   *   @param angle to rotate by (in degrees)
   *   @return boolean true on successful completion
   */
  bool rotate();
  /**
   *   @brief function to find closest frontier to the robot
   *
   *   @param the vector containing collection of frontier centroids
   *   @return the closest frontier to the robot
   */
  std::pair<double, double> findNearestFrontier(
      const std::vector<std::pair<double, double>> &frontierCentroidWorldSet);
  /**
   *   @brief function to execute frontier exploaration task
   *
   *   @param nothing
   *   @return boolean true on successful completion
   */
  bool startExploration();
};

#endif  // INCLUDE_FRONTIER_EXPLORATION_FRONTIER_EXPLORATION_NODE_HPP_
