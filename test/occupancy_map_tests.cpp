/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    occupancy_map_tests.cpp
 *  @author  rohithjayarajan
 *  @date 12/15/2018
 *  @version 0.1
 *
 *  @brief source file for testing OccupancyMap class
 *
 *  @section DESCRIPTION
 *
 *  source file which contains tests for OccupancyMap class
 */
// gtest header
#include <gtest/gtest.h>
// C++ system header
#include <cstdint>
#include <utility>
#include <vector>
// BOOST header
#include <boost/range/irange.hpp>
// user defined headers
#include "map_structure.hpp"
#include "occupancy_map.hpp"
// ROS headers
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

/**
 *  @brief class OccupancyMapTests
 * used for setup and teardown for all the tests
 */
class OccupancyMapTests : public ::testing::Test {
 public:
  // initialize MockControlSystemHelper object
  OccupancyMap classTests;
  nav_msgs::MapMetaData info;
  /**
   *   @brief Default constructor for OccupancyMapTests
   *
   *   @param nothing
   *   @return nothing
   */
  OccupancyMapTests() {}
  /**
   *   @brief Default destructor for OccupancyMapTests
   *
   *   @param nothing
   *   @return nothing
   */
  ~OccupancyMapTests() {}
  /**
   *   @brief setup up function to set up the tests with variables or objects
   *
   *   @param nothing
   *   @return nothing
   */
  void SetUp() {
    nav_msgs::OccupancyGridPtr dummyMap(new nav_msgs::OccupancyGrid);
    info.origin.position.x = 1;
    info.origin.position.y = 2;
    info.origin.position.z = 0;
    info.origin.orientation.x = 0;
    info.origin.orientation.y = 0;
    info.origin.orientation.z = 0;
    info.origin.orientation.w = 0;
    info.resolution = 0.03;
    info.width = 4;
    info.height = 5;

    dummyMap->info = info;
    dummyMap->data = {100, 100, 100, -1, 100, 100, -1, 0,
                      100, -1,  0,   0,  -1,  0,   0,  0};
    auto readStatus = classTests.readOccupancyGrid(dummyMap);
  }
  /**
   *   @brief tear down function for the tests to clear up the used
   * variables or objects
   *
   *   @param nothing
   *   @return nothing
   */
  void TearDown() {}
};

/**
 *   @brief test function to test resolution variable getter & setter method
 * OccupancyMap class
 *
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapReslution, the name of test
 */
TEST_F(OccupancyMapTests, getSetMapReslution) {
  ASSERT_NEAR(classTests.getResolution(), 0.03, 0.1);
}
/**
 *   @brief test function to test height variable getter & setter method
 * OccupancyMap class
 *
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapHeight, the name of test
 */
TEST_F(OccupancyMapTests, getSetMapHeight) {
  ASSERT_EQ(classTests.getHeight(), 5);
}
/**
 *   @brief test function to test width variable getter & setter method
 * OccupancyMap class
 *
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapWidth, the name of test
 */
TEST_F(OccupancyMapTests, getSetMapWidth) {
  ASSERT_EQ(classTests.getWidth(), 4);
}
/**
 *   @brief test function to test origin variable getter & setter method
 * OccupancyMap class
 *
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapMetadata, the name of test
 */
TEST_F(OccupancyMapTests, getSetMapOigin) {
  geometry_msgs::Pose check = classTests.getOrigin();
  ASSERT_EQ(check.position.x, 1);
  ASSERT_EQ(check.position.y, 2);
}
/**
 *   @brief test function to test isFrontier method OccupancyMap class
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapMetadata, the name of test
 */
TEST_F(OccupancyMapTests, testIsFrontier) {
  int x_ = 0;
  int y_ = 0;
  ASSERT_EQ(classTests.isFrontier(x_, y_), false);
}
/**
 *   @brief test function to test classifyFrontiers method OccupancyMap class
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapMetadata, the name of test
 */
TEST_F(OccupancyMapTests, testClassifyFrontiers) {
  ASSERT_EQ(classTests.classifyFrontiers(), 1);
}
/**
 *   @brief test function to test getFrontierCentroid method OccupancyMap class
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapMetadata, the name of test
 */
TEST_F(OccupancyMapTests, testFrontierCentroid) {
  std::vector<std::pair<double, double>> testFrontierCentersGrid;
  testFrontierCentersGrid.push_back(std::make_pair(0, 0));
  testFrontierCentersGrid.push_back(std::make_pair(2, 2));
  auto checkCentroid = std::make_pair(1, 1);
  ASSERT_EQ(classTests.classifyFrontiers(), 1);
}
/**
 *   @brief test function to test grid2world method OccupancyMap class
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapMetadata, the name of test
 */
TEST_F(OccupancyMapTests, testGrid2world) {
  std::vector<std::pair<uint32_t, uint32_t>> testFrontierCentersGrid;
  testFrontierCentersGrid.push_back(std::make_pair(0, 0));
  testFrontierCentersGrid.push_back(std::make_pair(2, 2));
  std::vector<std::pair<double, double>> testAnswer =
      classTests.grid2world(testFrontierCentersGrid);
  ASSERT_NEAR(testAnswer[0].first, 1, 0.1);
  ASSERT_NEAR(testAnswer[0].second, 2, 0.1);
  ASSERT_NEAR(testAnswer[1].first, 1.06, 0.1);
  ASSERT_NEAR(testAnswer[1].second, 2.06, 0.1);
}
/**
 *   @brief test function to test detectFrontiersCenterWorld method OccupancyMap
 * class
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param getSetMapMetadata, the name of test
 */
TEST_F(OccupancyMapTests, testDetectFrontiersCenterWorld) {
  std::vector<std::pair<double, double>> checkFrontierCenters;
  ASSERT_EQ(classTests.detectFrontiersCenterWorld(), checkFrontierCenters);
}
/**
 *   @brief test function to test readOccupancyGrid method OccupancyMap
 * class
 *
 *   @param OccupancyMapTests, the gtest framework
 *   @param testReadOccupancyGrid, the name of test
 */
TEST_F(OccupancyMapTests, testReadOccupancyGrid) {
  bool readStatus = true;
  info.origin.position.x = 1;
  info.origin.position.y = 2;
  info.origin.position.z = 0;
  info.origin.orientation.x = 0;
  info.origin.orientation.y = 0;
  info.origin.orientation.z = 0;
  info.origin.orientation.w = 0;
  info.resolution = 0.03;
  info.width = 4;
  info.height = 5;

  nav_msgs::OccupancyGridPtr dummyRead(new nav_msgs::OccupancyGrid);
  dummyRead->info = info;
  dummyRead->data = {100, 100, 100, -1, 100, 100, -1, 0,
                     100, -1,  0,   0,  -1,  0,   0,  0};
  ASSERT_EQ(classTests.readOccupancyGrid(dummyRead), readStatus);
}
