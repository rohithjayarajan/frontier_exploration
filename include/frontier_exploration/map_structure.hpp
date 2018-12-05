/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    map_structure.hpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief header file for MapStructure class
 *
 *  @section DESCRIPTION
 *
 *  header file which contains the declaration of MapStructure class
 *
 */

#ifndef INCLUDE_FRONTIER_EXPLORATION_MAP_STRUCTURE_HPP_
#define INCLUDE_FRONTIER_EXPLORATION_MAP_STRUCTURE_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <vector>
/**
 * @brief the declaration of MapStructure class
 *
 * Declaration of the variable and methods of MapStructure class
 */
class MapStructure {
 private:
  double resolution;
  unsigned int width;
  unsigned int height;
  geometry_msgs::Pose origin;
  bool isOnFrontier;
  unsigned int frontierClass;
  int x;
  int y;
  double data;

 public:
  /**
   *   @brief Default constructor for MapStructure
   *
   *   @param nothing
   *   @return nothing
   */
  MapStructure();
  /**
   *   @brief custom constructor for MapStructure
   *
   *   @param unsigned int value for resolution
   *   @param unsigned int value for width
   *   @param unsigned int value for height
   *   @param nothgeometry_msgs::Pose value for origin
   *   @return nothing
   */
  MapStructure(const double &resolution, unsigned const int &width,
               unsigned const int &height, const geometry_msgs::Pose &origin);
  /**
   *   @brief Default destructor for MapStructure
   *
   *   @param nothing
   *   @return nothing
   */
  ~MapStructure();
  /**
   *   @brief setter for resolution variable
   *
   *   @param double value of resolution
   *   @return nothing
   */
  void setResolution(const double &resolution_);
  /**
   *   @brief setter for width variable
   *
   *   @param unsigned int value of width
   *   @return nothing
   */
  void setWidth(unsigned const int &width_);
  /**
   *   @brief setter for height variable
   *
   *   @param unsigned int value of height
   *   @return nothing
   */
  void setHeight(unsigned const int &height_);
  /**
   *   @brief setter for origin variable
   *
   *   @param geometry_msgs::Pose value of origin
   *   @return nothing
   */
  void setOrigin(const geometry_msgs::Pose &origin_);
  /**
   *   @brief setter for isOnFrontier variable
   *
   *   @param bool value of isOnFrontier
   *   @return nothing
   */
  void setIsOnFrontier(const bool &isOnFrontier_);
  /**
   *   @brief setter for frontierClass variable
   *
   *   @param unsigned int value of frontierClass
   *   @return nothing
   */
  void setFrontierClass(unsigned const int &frontierClass_);
  /**
   *   @brief setter for map coordinate x variable
   *
   *   @param int value of x coordinate
   *   @return nothing
   */
  void setMapXCoordinate(const int &x_);
  /**
   *   @brief setter for map coordinate y variable
   *
   *   @param int value of y coordinate
   *   @return nothing
   */
  void setMapYCoordinate(const int &y_);
  /**
   *   @brief setter for map data
   *
   *   @param double value of map data
   *   @return nothing
   */
  void setMapData(const double &data_);
  /**
   *   @brief getter for resolution variable
   *
   *   @param nothing
   *   @return double value of resolution
   */
  double getResolution();
  /**
   *   @brief getter for width variable
   *
   *   @param nothing
   *   @return unsigned int value of width
   */
  unsigned int getWidth();
  /**
   *   @brief getter for height variable
   *
   *   @param nothing
   *   @return unsigned int value of height
   */
  unsigned int getHeight();
  /**
   *   @brief getter for origin variable
   *
   *   @param nothing
   *   @return geometry_msgs::Pose value of origin
   */
  geometry_msgs::Pose getOrigin();
  /**
   *   @brief getter for isOnFrontier variable
   *
   *   @param nothing
   *   @return bool value of isOnFrontier
   */
  bool getIsOnFrontier();
  /**
   *   @brief getter for frontierClass variable
   *
   *   @param nothing
   *   @return unsigned int value of frontierClass
   */
  unsigned int getFrontierClass();
  /**
   *   @brief getter for map coordinate x variable
   *
   *   @param nothing
   *   @return int value of x coordinate
   */
  int getMapXCoordinate();
  /**
   *   @brief getter for map coordinate y variable
   *
   *   @param nothing
   *   @return int value of y coordinate
   */
  int getMapYCoordinate();
  /**
   *   @brief getter for map data
   *
   *   @param nothing
   *   @return double value of map data
   */
  double getMapData();
};
#endif  // INCLUDE_FRONTIER_EXPLORATION_MAP_STRUCTURE_HPP_
