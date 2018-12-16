/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    occupancy_map.cpp
 *  @author  rohithjayarajan
 *  @date 16/2/2018
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
#include "ros/ros.h"

// value for default class
const int DEF_CLASS = -255;
// value for minimum frontiers in a frontier group
const int MIN_FRONTIER_SIZE = 22;
OccupancyMap::OccupancyMap() {
  // set the status of map is loaded to false
  isMapLoaded = false;
  // initialize resolution variable
  resolution = 0.0;
  // initialize width variable
  width = 0;
  // initialize height variable
  height = 0;
}

OccupancyMap::~OccupancyMap() {}

bool OccupancyMap::loadMapMetadata(
    const nav_msgs::OccupancyGrid::ConstPtr &grid_msg) {
  // set the value of resolution
  resolution = grid_msg->info.resolution;
  // set the value of width
  width = grid_msg->info.width;
  // set the value of height
  height = grid_msg->info.height;
  // set the value of origin
  origin = grid_msg->info.origin;
  // return true on successful completion of function
  return true;
}

double OccupancyMap::getResolution() {
  // return the value of resolution
  return resolution;
}

uint32_t OccupancyMap::getHeight() {
  // return the value of height
  return height;
}

uint32_t OccupancyMap::getWidth() {
  // return the value of width
  return width;
}

geometry_msgs::Pose OccupancyMap::getOrigin() {
  // return the value of origin
  return origin;
}

bool OccupancyMap::readOccupancyGrid(
    const nav_msgs::OccupancyGrid::ConstPtr &grid_msg) {
  // set iterator to zero to load data from occupancy grid
  double idx = 0;
  // create object of class MapStructure
  MapStructure mapCell;
  // declare variable to push value into occupancy grid
  std::vector<MapStructure> store1Dim;
  // conditions for updating occupancy grid
  if (!isMapLoaded) {
    // load the map meta data information
    loadMapMetadata(grid_msg);
    // convert the 1D occupancy grid array into 2D array and store it
    for (int h : boost::irange(0, static_cast<int>(height))) {
      for (int w : boost::irange(0, static_cast<int>(width))) {
        mapCell.setMapXYCoordinate(h, w);
        mapCell.setMapData(grid_msg->data[idx]);
        store1Dim.push_back(mapCell);
        idx++;
      }
      occupancyMap.push_back(store1Dim);
      store1Dim.clear();
      isMapLoaded = true;
    }
  } else {
    // check if the map meta data has to be updated and update map accordingly
    if ((width == grid_msg->info.width) && (height == grid_msg->info.height) &&
        resolution == grid_msg->info.resolution &&
        origin.position.x == grid_msg->info.origin.position.x &&
        origin.position.y == grid_msg->info.origin.position.y) {
      for (int h : boost::irange(0, static_cast<int>(height))) {
        for (int w : boost::irange(0, static_cast<int>(width))) {
          occupancyMap[h][w].setMapData(grid_msg->data[idx]);
          idx++;
        }
      }
    } else {
      // load the map meta data information and then update the map
      loadMapMetadata(grid_msg);
      for (int h : boost::irange(0, static_cast<int>(height))) {
        for (int w : boost::irange(0, static_cast<int>(width))) {
          mapCell.setMapXYCoordinate(h, w);
          mapCell.setMapData(grid_msg->data[idx]);
          store1Dim.push_back(mapCell);
          idx++;
        }
        occupancyMap.push_back(store1Dim);
        store1Dim.clear();
      }
    }
  }
  // return true on successful completion of this function
  return true;
}

bool OccupancyMap::isFrontier(const int &x_, const int &y_) {
  // check around neighbors to evaluate if this point is a frontier
  for (int h : boost::irange(x_ - 1, x_ + 2)) {
    for (int w : boost::irange(y_ - 1, y_ + 2)) {
      if (h >= 0 && w >= 0 && h < height && w < width &&
          occupancyMap[h][w].getMapData() == -1) {
        return true;
      }
    }
  }
  return false;
}

int OccupancyMap::getFrontiersPoints() {
  // set number of frontier points to zero
  int frontier = 0;
  // variable to perform check if point is on frontier
  bool isOnFrontier_;
  // get frontier points
  for (int h : boost::irange(0, static_cast<int>(height))) {
    for (int w : boost::irange(0, static_cast<int>(width))) {
      if (occupancyMap[h][w].getMapData() == 0) {
        isOnFrontier_ = false;
        isOnFrontier_ = isFrontier(h, w);
        occupancyMap[h][w].setIsOnFrontier(isOnFrontier_);
      }
      if (isOnFrontier_) {
        frontier++;
      }
      // update the frontier class to a default constant value for all points
      occupancyMap[h][w].setFrontierClass(DEF_CLASS);
    }
  }
  return frontier;
}

int OccupancyMap::classifyFrontiers() {
  // iterator to go over all the frontier classes
  int iterFrontierClass = 1;
  // check and group frontiers into clusters
  for (int h : boost::irange(0, static_cast<int>(height))) {
    for (int w : boost::irange(0, static_cast<int>(width))) {
      if (occupancyMap[h][w].getIsOnFrontier()) {
        if ((h - 1 >= 0) && (w - 1 >= 0) &&
            occupancyMap[h - 1][w - 1].getFrontierClass() != DEF_CLASS &&
            occupancyMap[h][w].getFrontierClass() == DEF_CLASS) {
          auto tempFrontierClass =
              occupancyMap[h - 1][w - 1].getFrontierClass();
          occupancyMap[h][w].setFrontierClass(tempFrontierClass);
        } else if (((h - 1 >= 0) && (w - 1 >= 0) &&
                    occupancyMap[h - 1][w].getFrontierClass() == DEF_CLASS &&
                    occupancyMap[h][w - 1].getFrontierClass() == DEF_CLASS) ||
                   ((h - 1 >= 0) && (w - 1 < 0) &&
                    occupancyMap[h - 1][w].getFrontierClass() == DEF_CLASS) ||
                   ((h - 1 < 0) && (w - 1 >= 0) &&
                    occupancyMap[h][w - 1].getFrontierClass() == DEF_CLASS)) {
          occupancyMap[h][w].setFrontierClass(iterFrontierClass);
          // increment iterator for new class detected
          iterFrontierClass++;
        } else if ((h - 1 >= 0) &&
                   occupancyMap[h - 1][w].getFrontierClass() != DEF_CLASS &&
                   occupancyMap[h][w].getFrontierClass() == DEF_CLASS) {
          auto tempFrontierClass = occupancyMap[h - 1][w].getFrontierClass();
          occupancyMap[h][w].setFrontierClass(tempFrontierClass);
        } else if ((w - 1 >= 0) &&
                   occupancyMap[h][w - 1].getFrontierClass() != DEF_CLASS &&
                   occupancyMap[h][w].getFrontierClass() == DEF_CLASS) {
          auto tempFrontierClass = occupancyMap[h][w - 1].getFrontierClass();
          occupancyMap[h][w].setFrontierClass(tempFrontierClass);
        } else if ((h - 1 >= 0) && (w >= 0) && (w < width) &&
                   occupancyMap[h - 1][w + 1].getFrontierClass() != DEF_CLASS &&
                   occupancyMap[h][w].getFrontierClass() == DEF_CLASS) {
          auto tempFrontierClass = occupancyMap[h][w - 1].getFrontierClass();
          occupancyMap[h][w].setFrontierClass(tempFrontierClass);
          // increment iterator for new class detected
          iterFrontierClass++;
        }
      }
    }
  }
  return iterFrontierClass;
}

std::pair<double, double> OccupancyMap::getFrontierCentroid(
    std::vector<std::pair<double, double>> &frontierCentersGrid) {
  // intialize variable to compute sum of X coordinates
  double sumWorldX = 0;
  // intialize variable to compute sum of Y coordinates
  double sumWorldY = 0;
  // get sum of all X and Y coordinate values respectively
  for (auto frontierPair : frontierCentersGrid) {
    sumWorldX += frontierPair.first;
    sumWorldY += frontierPair.second;
  }
  // get average value of X and Y coordinate values respectively
  double centroidX = sumWorldX / frontierCentersGrid.size();
  double centroidY = sumWorldY / frontierCentersGrid.size();
  // return the frontier group centroid value
  return std::make_pair(centroidX, centroidY);
}

std::vector<std::pair<double, double>> OccupancyMap::grid2world(
    std::vector<std::pair<uint32_t, uint32_t>> &frontierCentersGrid) {
  // decalare variable which is to be returned, the map cell values in world
  // coordinates
  std::vector<std::pair<double, double>> frontierCentersWorld;
  for (auto frontierPair : frontierCentersGrid) {
    // get world coordiante X position
    auto worldX = origin.position.x +
                  (resolution * static_cast<double>(frontierPair.second));
    // get world coordiante Y position
    auto worldY = origin.position.y +
                  (resolution * static_cast<double>(frontierPair.first));
    frontierCentersWorld.push_back(std::make_pair(worldX, worldY));
  }
  // return the converted grid cell values in world cooridinates
  return frontierCentersWorld;
}

std::vector<std::pair<double, double>>
OccupancyMap::detectFrontiersCenterWorld() {
  // get the frontier points and the number of frontier points
  auto numberOfFrontiers = getFrontiersPoints();
  // print number of frontier points detected
  ROS_INFO_STREAM(
      "The number of frontier points detected are: " << numberOfFrontiers);
  // classify the frontier and get the number of frontier groups
  auto iterFrontierClass = classifyFrontiers();
  // print number of frontier groups detected
  ROS_INFO_STREAM(
      "The number of frontier groups detected are: " << iterFrontierClass);
  // the set of frontier centroid points in world coordinate
  std::vector<std::pair<double, double>> frontierCentroidWorldSet;
  // an element of the set frontierCentroidWorldSet
  std::pair<double, double> frontierCentroidWorld;
  // the frontier pairs in grid coordinates
  std::vector<std::pair<uint32_t, uint32_t>> frontiers;
  // the frontier pairs in world coordinates
  std::vector<std::pair<double, double>> frontiersWorld;
  // process each frontier group
  for (auto iter : boost::irange(1, iterFrontierClass)) {
    for (auto frontierSet : occupancyMap) {
      for (auto frontierElement : frontierSet) {
        if (frontierElement.getFrontierClass() == iter) {
          frontiers.push_back(frontierElement.getMapXYCoordinate());
        }
      }
    }
    // convert the frontiers from grid coordinates to world coordinates
    frontiersWorld = grid2world(frontiers);
    // discard the frontier groups which have number of elements below a certain
    // threshold
    if (frontiers.size() > MIN_FRONTIER_SIZE) {
      // centroid of the frontier group processed
      frontierCentroidWorld = getFrontierCentroid(frontiersWorld);
      // store this centroid in the set of all frontier centroids
      frontierCentroidWorldSet.push_back(frontierCentroidWorld);
    }
    // clear vector for next iteration
    frontiers.clear();
    // clear vector for next iteration
    frontiersWorld.clear();
  }
  // return the collection of all frontier group centroids
  return frontierCentroidWorldSet;
}
