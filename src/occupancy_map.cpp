/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    occupancy_map.cpp
 *  @author  rohithjayarajan
 *  @date 14/2/2018
 *  @version 1.1
 *
 *  @brief source file for OccupancyMap class
 *
 *  @section DESCRIPTION
 *
 *  source file which contains the defintion of OccupancyMap class
 *
 */
#include "occupancy_map.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <boost/range/irange.hpp>
#include <cstdint>
#include <utility>
#include <vector>
#include "map_structure.hpp"
#include "nav_msgs/OccupancyGrid.h"

OccupancyMap::OccupancyMap() {}

OccupancyMap::~OccupancyMap() {}

bool OccupancyMap::loadMapMetadata(
    const nav_msgs::OccupancyGrid::ConstPtr &grid_msg) {
  return true;
}

double OccupancyMap::getResolution() { return 0.0; }

uint32_t OccupancyMap::getHeight() { return 1; }

uint32_t OccupancyMap::getWidth() { return 1; }

geometry_msgs::Pose OccupancyMap::getOrigin() {
  geometry_msgs::Pose a;
  return a;
}

bool OccupancyMap::readOccupancyGrid(
    const nav_msgs::OccupancyGrid::ConstPtr &grid_msg) {
  return true;
}

bool OccupancyMap::isFrontier(const int &x_, const int &y_) { return true; }

int OccupancyMap::getFrontiersPoints() { return 1; }

int OccupancyMap::classifyFrontiers() { return 1; }

std::pair<double, double> OccupancyMap::getFrontierCentroid(
    std::vector<std::pair<double, double>> &frontierCentersGrid) {
  std::pair<double, double> a;
  return a;
}

std::vector<std::pair<double, double>> OccupancyMap::grid2world(
    std::vector<std::pair<uint32_t, uint32_t>> &frontierCentersGrid) {
  std::vector<std::pair<double, double>> a;
  return a;
}

std::vector<std::pair<double, double>>
OccupancyMap::detectFrontiersCenterWorld() {
  std::vector<std::pair<double, double>> a;
  return a;
}
