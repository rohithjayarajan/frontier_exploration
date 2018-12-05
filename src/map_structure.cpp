/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    map_structure.cpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief source file for MapStructure class
 *
 *  @section DESCRIPTION
 *
 *  header file which contains the definiton of MapStructure class
 */
#include "map_structure.hpp"
/**
 *   @brief Default constructor for MapStructure
 *
 *   @param nothing
 *   @return nothing
 */
MapStructure::MapStructure() {}
/**
 *   @brief custom constructor for MapStructure
 *
 *   @param unsigned int value for resolution
 *   @param unsigned int value for width
 *   @param unsigned int value for height
 *   @param nothgeometry_msgs::Pose value for origin
 *   @return nothing
 */
MapStructure::MapStructure(double &resolution, unsigned const int &width,
                           unsigned const int &height,
                           const geometry_msgs::Pose &origin) {}
/**
 *   @brief Default destructor for MapStructure
 *
 *   @param nothing
 *   @return nothing
 */
MapStructure::~MapStructure() {}
/**
 *   @brief setter for resolution variable
 *
 *   @param double value of resolution
 *   @return nothing
 */
void MapStructure::setResolution(const double &resolution_) {}
/**
 *   @brief setter for width variable
 *
 *   @param unsigned int value of width
 *   @return nothing
 */
void MapStructure::setWidth(unsigned const int &width_) {}
/**
 *   @brief setter for height variable
 *
 *   @param unsigned int value of height
 *   @return nothing
 */
void MapStructure::setHeight(unsigned const int &height_) {}
/**
 *   @brief setter for origin variable
 *
 *   @param geometry_msgs::Pose value of origin
 *   @return nothing
 */
void MapStructure::setOrigin(const geometry_msgs::Pose &origin_) {}
/**
 *   @brief setter for isOnFrontier variable
 *
 *   @param bool value of isOnFrontier
 *   @return nothing
 */
void MapStructure::setIsOnFrontier(const bool &isOnFrontier_) {}
/**
 *   @brief setter for frontierClass variable
 *
 *   @param unsigned int value of frontierClass
 *   @return nothing
 */
void MapStructure::setFrontierClass(unsigned const int &frontierClass_) {}
/**
 *   @brief setter for map coordinate x variable
 *
 *   @param int value of x coordinate
 *   @return nothing
 */
void MapStructure::setMapXCoordinate(const int &x_) {}
/**
 *   @brief setter for map coordinate y variable
 *
 *   @param int value of y coordinate
 *   @return nothing
 */
void MapStructure::setMapYCoordinate(const int &y_) {}
/**
 *   @brief setter for map data
 *
 *   @param double value of map data
 *   @return nothing
 */
void MapStructure::setMapData(const double &data_) {}
/**
 *   @brief getter for resolution variable
 *
 *   @param nothing
 *   @return unsigned int value of resolution
 */
double MapStructure::getResolution() { return 0.0; }
/**
 *   @brief getter for width variable
 *
 *   @param nothing
 *   @return unsigned int value of width
 */
unsigned int MapStructure::getWidth() { return 0; }
/**
 *   @brief getter for height variable
 *
 *   @param nothing
 *   @return unsigned int value of height
 */
unsigned int MapStructure::getHeight() { return 0; }
/**
 *   @brief getter for origin variable
 *
 *   @param nothing
 *   @return geometry_msgs::Pose value of origin
 */
geometry_msgs::Pose MapStructure::getOrigin() {
  geometry_msgs::Pose a;
  return a;
}
/**
 *   @brief getter for isOnFrontier variable
 *
 *   @param nothing
 *   @return bool value of isOnFrontier
 */
bool MapStructure::getIsOnFrontier() { return true; }
/**
 *   @brief getter for frontierClass variable
 *
 *   @param nothing
 *   @return unsigned int value of frontierClass
 */
unsigned int MapStructure::getFrontierClass() { return 0; }
/**
 *   @brief getter for map coordinate x variable
 *
 *   @param nothing
 *   @return int value of x coordinate
 */
int MapStructure::getMapXCoordinate() { return 0; }
/**
 *   @brief getter for map coordinate y variable
 *
 *   @param nothing
 *   @return int value of y coordinate
 */
int MapStructure::getMapYCoordinate() { return 0; }
/**
 *   @brief getter for map data
 *
 *   @param nothing
 *   @return double value of map data
 */
double MapStructure::getMapData() { return 0.0; }
