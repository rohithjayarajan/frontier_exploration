/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    frontier_exploration_node.cpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief source file for FrontierExplore class
 *
 *  @section DESCRIPTION
 *
 *  header file which contains the defintion of FrontierExplore
 * class
 *
 */
#include "frontier_exploration_node.hpp"
/**
 *   @brief Default constructor for FrontierExplore
 *
 *   @param nothing
 *   @return nothing
 */
FrontierExplore::FrontierExplore() {}
/**
 *   @brief Default destructor for FrontierExplore
 *
 *   @param nothing
 *   @return nothing
 */
FrontierExplore::~FrontierExplore() {}
/**
 *   @brief callback function for to monitor collision threat based on laser
 * scan messages
 *
 *   @param boost shared pointer to sensor_msgs::LaserScan
 *   @return nothing
 */
void FrontierExplore::laserScanCallback(
    const sensor_msgs::LaserScan::ConstPtr &msg) {}
/**
 *   @brief function to read the occupancy grid
 *
 *   @param nothing
 *   @return nothing
 */
void FrontierExplore::readOccupancyGrid(
    const nav_msgs::OccupancyGrid::ConstPtr &grid_msg) {}
/**
 *   @brief function to check if the point is a frontier point
 *
 *   @param int values of index
 *   @return boolean value
 */
bool FrontierExplore::isFrontier(const int &x_, const int &y_) { return true; }
/**
 *   @brief function to get frontier points
 *
 *   @param nothing
 *   @return nothing
 */
void FrontierExplore::getFrontiersPoints() {}
/**
 *   @brief function to group frontiers
 *
 *   @param nothing
 *   @return nothing
 */
void FrontierExplore::classifyFrontiers() {}
/**
 *   @brief function to get frontiers of the map
 *
 *   @param nothing
 *   @return nothing
 */
void FrontierExplore::detectFrontiers() {}
/**
 *   @brief function to get robot pose
 *
 *   @param boost shared pointer to nav_msgs::Odometry
 *   @return nothing
 */
void FrontierExplore::updateRobotPose(
    const nav_msgs::Odometry::ConstPtr &odom_msg) {}
/**
 *   @brief function to get goal pose from frontiers
 *
 *   @param nothing
 *   @return nothing
 */
void FrontierExplore::updateGoalPose() {}
/**
 *   @brief function to rotate the turtlebot by a value (in degrees) at a
 * fixed point
 *
 *   @param angle to rotate by (in degrees
 *   @return nothing
 */
void FrontierExplore::rotate(const double &angleToRotate) {}
/**
 *   @brief function to execute frontier exploaration task
 *
 *   @param nothing
 *   @return nothing
 */
void FrontierExplore::startExploration() {}
