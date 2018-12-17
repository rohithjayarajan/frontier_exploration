/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    map_structure_tests.cpp
 *  @author  rohithjayarajan
 *  @date 12/15/2018
 *  @version 1.1
 *
 *  @brief source file for testing MapStructure class
 *
 *  @section DESCRIPTION
 *
 *  source file which contains tests for MapStructure class
 */
#include "map_structure.hpp"
// C++ system header
#include <gtest/gtest.h>
#include <cstdint>
#include <utility>
/**
 *   @brief test function to test isOnFrontier variable getter & setter method
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetIsOnFrontier, the name of test
 */
TEST(MapStructureTests, getSetIsOnFrontier) {
  MapStructure testObject1;
  bool isOnFrontier = true;
  testObject1.setIsOnFrontier(isOnFrontier);
  EXPECT_EQ(testObject1.getIsOnFrontier(), isOnFrontier);
}
/**
 *   @brief test function to test frontierClass variable getter & setter method
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetfrontierClass, the name of test
 */
TEST(MapStructureTests, getSetfrontierClass) {
  MapStructure testObject2;
  int frontierClass = 5;
  testObject2.setFrontierClass(frontierClass);
  EXPECT_EQ(testObject2.getFrontierClass(), frontierClass);
}

/**
 *   @brief test function to test x and y variable getter & setter method
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetMapXYCoordinate, the name of test
 */
TEST(MapStructureTests, getSetMapXYCoordinate) {
  MapStructure testObject3;
  uint32_t x_ = 5;
  uint32_t y_ = 10;
  testObject3.setMapXYCoordinate(x_, y_);
  EXPECT_EQ(testObject3.getMapXYCoordinate(), std::make_pair(x_, y_));
}

/**
 *   @brief test function to test data variable getter & setter method
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetMapData, the name of test
 */
TEST(MapStructureTests, getSetMapData) {
  MapStructure testObject4;
  int8_t data_ = 1;
  testObject4.setMapData(data_);
  EXPECT_EQ(testObject4.getMapData(), data_);
}
