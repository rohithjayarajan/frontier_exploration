/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    map_structure.cpp
 *  @author  rohithjayarajan
 *  @date 14/2/2018
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
#include <cstdint>
#include <utility>

MapStructure::MapStructure() {}

MapStructure::~MapStructure() {}

void MapStructure::setIsOnFrontier(const bool &isOnFrontier_) {}

void MapStructure::setFrontierClass(const int &frontierClass_) {}

void MapStructure::setMapXYCoordinate(const uint32_t &x_, const uint32_t &y_) {}

void MapStructure::setMapData(const int8_t &data_) {}

bool MapStructure::getIsOnFrontier() { return false; }

int MapStructure::getFrontierClass() { return 1; }

std::pair<uint32_t, uint32_t> MapStructure::getMapXYCoordinate() {
  std::pair<uint32_t, uint32_t> a;
  return a;
}

int8_t MapStructure::getMapData() { return 1; }
