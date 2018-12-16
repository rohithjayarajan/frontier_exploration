/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    map_structure.cpp
 *  @author  rohithjayarajan
 *  @date 16/2/2018
 *  @version 1.1
 *
 *  @brief source file for MapStructure class
 *
 *  @section DESCRIPTION
 *
 *  source file which contains the defintion of MapStructure class
 *
 */
#include "map_structure.hpp"
// C++ system header
#include <cstdint>
#include <utility>

MapStructure::MapStructure() {}

MapStructure::~MapStructure() {}

void MapStructure::setIsOnFrontier(const bool &isOnFrontier_) {
  // set the value of variable isOnFrontier
  isOnFrontier = isOnFrontier_;
}

void MapStructure::setFrontierClass(const int &frontierClass_) {
  // set the value of variable frontierClass
  frontierClass = frontierClass_;
}

void MapStructure::setMapXYCoordinate(const uint32_t &x_, const uint32_t &y_) {
  // set the value of variable x
  x = x_;
  // set the value of variable y
  y = y_;
}

void MapStructure::setMapData(const int8_t &data_) {
  // set the value of variable data
  data = data_;
}

bool MapStructure::getIsOnFrontier() {
  // return the value of variable isOnFrontier
  return isOnFrontier;
}

int MapStructure::getFrontierClass() {
  // return the value of variable frontierClass
  return frontierClass;
}

std::pair<uint32_t, uint32_t> MapStructure::getMapXYCoordinate() {
  std::pair<uint32_t, uint32_t> XYCoordinate(x, y);
  // return the value of variable XYCoordinate
  return XYCoordinate;
}

int8_t MapStructure::getMapData() {
  // return the value of variable data
  return data;
}
