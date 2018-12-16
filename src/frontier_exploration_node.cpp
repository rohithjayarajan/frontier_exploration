/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    frontier_exploration_node.cpp
 *  @author  rohithjayarajan
 *  @date 14/2/2018
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

FrontierExplore::FrontierExplore() {}

FrontierExplore::~FrontierExplore() {}

bool FrontierExplore::updateMap(
    const nav_msgs::OccupancyGrid::ConstPtr &grid_msg) {
  return true;
}

bool FrontierExplore::updateRobotPose() { return true; }

bool FrontierExplore::updateGoalPose() { return true; }

bool FrontierExplore::rotate() { return true; }

std::pair<double, double> FrontierExplore::findNearestFrontier(
    const std::vector<std::pair<double, double>> &frontierCentroidWorldSet) {
  std::pair<double, double> a;
  return a;
}

bool FrontierExplore::startExploration() { return true; }
