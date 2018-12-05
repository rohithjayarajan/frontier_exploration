/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    helper_functions.cpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief source file for helper functions
 *
 *  @section DESCRIPTION
 *
 *  header file which contains the definition of helper functions used in
 * HelperFunctions class
 *
 */
#include "helper_functions.hpp"
/**
 *   @brief Default constructor for HelperFunctions
 *
 *   @param nothing
 *   @return nothing
 */
HelperFunctions::HelperFunctions() {}
/**
 *   @brief Default destructor for HelperFunctions
 *
 *   @param nothing
 *   @return nothing
 */
HelperFunctions::~HelperFunctions() {}
/**
 *   @brief function to compute distance between two pairs of points
 *
 *   @param two pairs of 2D points
 *   @return double value of distance between two pairs of points
 */
double HelperFunctions::computeDistance(const geometry_msgs::Point &p1,
                                        const geometry_msgs::Point &p2) {
  return 0.0;
}
/**
 *   @brief unction to compute gradient between two pairs of points in
 *
 *   @param two pairs of 2D points
 *   @return double value of distance between two pairs of points
 */
double HelperFunctions::computeGradient(const geometry_msgs::Point &p1,
                                        const geometry_msgs::Point &p2) {
  return 0.0;
}
/**
 *   @brief function to convert radians to degrees
 *
 *   @param angle in radians of type double
 *   @return angle in degrees of type double
 */
double HelperFunctions::rad2deg(const double &angle) { return 0.0; }
/**
 *   @brief function to convert degrees to radians
 *
 *   @param angle in degrees of type double
 *   @return angle in radians of type double
 */
double HelperFunctions::deg2rad(const double &angle) { return 0.0; }
