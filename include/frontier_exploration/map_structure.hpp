/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    map_structure.hpp
 *  @author  rohithjayarajan
 *  @date 14/2/2018
 *  @version 1.1
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

#include <cstdint>
#include <utility>
/**
 * @brief the declaration of MapStructure class
 *
 * Declaration of the variable and methods of MapStructure class
 */
class MapStructure {
 private:
  // variable which tells if the grid cell is on a frontier
  bool isOnFrontier;
  // variable which gives the class number of the frontier
  int frontierClass;
  // variable to hold x value of grid poosition
  uint32_t x;
  // variable to hold x value of grid poosition
  uint32_t y;
  // variable to hold probability value of obstacle given by occupancy grid
  int8_t data;

 public:
  /**
   *   @brief Default constructor for MapStructure
   *
   *   @param nothing
   *   @return nothing
   */
  MapStructure();
  /**
   *   @brief Default destructor for MapStructure
   *
   *   @param nothing
   *   @return nothing
   */
  ~MapStructure();
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
   *   @param int value of frontierClass
   *   @return nothing
   */
  void setFrontierClass(const int &frontierClass_);
  /**
   *   @brief setter for map coordinate x and y variable
   *
   *   @param uint32_t values of x and y coordinate
   *   @return nothing
   */
  void setMapXYCoordinate(const uint32_t &x_, const uint32_t &y_);
  /**
   *   @brief setter for map data
   *
   *   @param int8_t value of map data
   *   @return nothing
   */
  void setMapData(const int8_t &data_);
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
   *   @return int value of frontierClass
   */
  int getFrontierClass();
  /**
   *   @brief getter for map coordinate x and y variable
   *
   *   @param nothing
   *   @return std::pair<uint32_t, uint32_t> value of x and y coordinate
   */
  std::pair<uint32_t, uint32_t> getMapXYCoordinate();
  /**
   *   @brief getter for map data
   *
   *   @param nothing
   *   @return int8_t value of map data
   */
  int8_t getMapData();
};
#endif  // INCLUDE_FRONTIER_EXPLORATION_MAP_STRUCTURE_HPP_
