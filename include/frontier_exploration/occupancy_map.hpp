/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    occupancy_map.hpp
 *  @author  rohithjayarajan
 *  @date 14/2/2018
 *  @version 1.1
 *
 *  @brief header file for OccupancyMap class
 *
 *  @section DESCRIPTION
 *
 *  header file which contains the declaration of OccupancyMap class
 *
 */
#ifndef INCLUDE_FRONTIER_EXPLORATION_OCCUPANCY_MAP_HPP_
#define INCLUDE_FRONTIER_EXPLORATION_OCCUPANCY_MAP_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <boost/range/irange.hpp>
#include <cstdint>
#include <utility>
#include <vector>
#include "map_structure.hpp"
#include "nav_msgs/OccupancyGrid.h"
/**
 * @brief the declaration of OccupancyMap class
 *
 * Declaration of the variable and methods of OccupancyMap class
 */
class OccupancyMap {
 private:
  // variable to check if the map is loaded
  bool isMapLoaded;
  // variable to hold resolution of occupancy grid
  double resolution;
  // variable to hold width of occupancy grid
  uint32_t width;
  // variable to hold height of occupancy grid
  uint32_t height;
  // variable to hold origin of occupancy grid
  geometry_msgs::Pose origin;
  // data structure containing the occupancy grid map
  std::vector<std::vector<MapStructure>> occupancyMap;

 public:
  /**
   *   @brief Default constructor for OccupancyMap
   *
   *   @param nothing
   *   @return nothing
   */
  OccupancyMap();
  /**
   *   @brief Default destructor for OccupancyMap
   *
   *   @param nothing
   *   @return nothing
   */
  ~OccupancyMap();
  /**
   *   @brief function to load the map metadata information
   *
   *   @param boost shared pointer to nav_msgs::OccupancyGrid
   *   @return boolean true on successful completion
   */
  bool loadMapMetadata(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg);
  /**
   *   @brief getter for resolution variable
   *
   *   @param nothing
   *   @return double value of resolution
   */
  double getResolution();
  /**
   *   @brief getter for height variable
   *
   *   @param nothing
   *   @return uint32_t value of height
   */
  uint32_t getHeight();
  /**
   *   @brief getter for width variable
   *
   *   @param nothing
   *   @return uint32_t value of width
   */
  uint32_t getWidth();
  /**
   *   @brief getter for origin variable
   *
   *   @param nothing
   *   @return geometry_msgs::Pose value of origin
   */
  geometry_msgs::Pose getOrigin();
  /**
   *   @brief function to read the occupancy grid
   *
   *   @param nothing
   *   @return boolean true on successful completion
   */
  bool readOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg);
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
   *   @param int value of number of frontier points
   *   @return nothing
   */
  int getFrontiersPoints();
  /**
   *   @brief function to group frontiers
   *
   *   @param nothing
   *   @return number of frontier clusters
   */
  int classifyFrontiers();
  /**
   *   @brief function to get centroids of frontiers of the
   * map
   *
   *   @param vector of frontier points of one cluster in world coordinates
   *   @return vector of centroid x,y coordinate pairs in world coordinates of
   * type double
   */
  std::pair<double, double> getFrontierCentroid(
      std::vector<std::pair<double, double>> &frontierCentersGrid);
  /**
   *   @brief function to convert grid coordinates to world coordinates
   *
   *   @param vector of frontier points in grid coordinates
   *   @return vector of frontier points in world coordinates
   */
  std::vector<std::pair<double, double>> grid2world(
      std::vector<std::pair<uint32_t, uint32_t>> &frontierCentersGrid);
  /**
   *   @brief function to get frontiers of the map
   *
   *   @param nothing
   *   @return vector of frontier centroid x,y coordinate pairs in world frame
   * of type double
   */
  std::vector<std::pair<double, double>> detectFrontiersCenterWorld();
};

#endif  // INCLUDE_FRONTIER_EXPLORATION_OCCUPANCY_MAP_HPP_
