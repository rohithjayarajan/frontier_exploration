/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    helper_functions.hpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief header file for helper functions
 *
 *  @section DESCRIPTION
 *
 *  header file which contains the declaration of helper functions used in
 * FrontierExplore class
 *
 */

#ifndef INCLUDE_FRONTIER_EXPLORATION_HELPER_FUNCTIONS_HPP_
#define INCLUDE_FRONTIER_EXPLORATION_HELPER_FUNCTIONS_HPP_
// C++ header
// #include <cmath>
// #include <sstream>
// #include <string>
// #include <utility>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <vector>
/**
 * @brief the declaration of HelperFunctions class
 *
 * Declaration of the variable and methods of HelperFunctions class which
 * consists of helper functions
 */
class HelperFunctions {
 public:
  /**
   *   @brief Default constructor for HelperFunctions
   *
   *   @param nothing
   *   @return nothing
   */
  HelperFunctions();
  /**
   *   @brief Default destructor for HelperFunctions
   *
   *   @param nothing
   *   @return nothing
   */
  ~HelperFunctions();
  /**
   *   @brief function to compute distance between two pairs of points
   *
   *   @param two pairs of 2D points
   *   @return double value of distance between two pairs of points
   */
  double computeDistance(const geometry_msgs::Point &p1,
                         const geometry_msgs::Point &p2);
  /**
   *   @brief function to compute gradient between two pairs of points
   *
   *   @param two pairs of 2D points
   *   @return double value of distance between two pairs of points
   */
  double computeGradient(const geometry_msgs::Point &p1,
                         const geometry_msgs::Point &p2);
  /**
   *   @brief function to convert radians to degrees
   *
   *   @param angle in radians of type double
   *   @return angle in degrees of type double
   */
  double rad2deg(const double &angle);
  /**
   *   @brief function to convert degrees to radians
   *
   *   @param angle in degrees of type double
   *   @return angle in radians of type double
   */
  double deg2rad(const double &angle);
};
#endif  // INCLUDE_FRONTIER_EXPLORATION_HELPER_FUNCTIONS_HPP_
