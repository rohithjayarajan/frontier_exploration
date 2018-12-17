/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    frontier_exploration_node_tests.cpp
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
#include "frontier_exploration_node.hpp"
// C++ system header
#include <cstdint>
#include <utility>
#include <vector>
// BOOST header
#include <boost/range/irange.hpp>
// gtest header
#include <gtest/gtest.h>
// user defined header
#include "occupancy_map.hpp"
// ROS header
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

/**
 *  @brief class PublisherTests
 * used for testing publisher
 */
class PublisherTests {
 public:
  void vel_Tester(const geometry_msgs::Twist::ConstPtr &vel_) {}
};
/**
 *  @brief class OccupancyMapTests
 * used for setup and teardown for all the tests
 */
class FrontierExploreTests : public ::testing::Test {
 public:
  ros::NodeHandle testnh;
  FrontierExplore testBot;
  PublisherTests pubTest;
  /**
   *   @brief Default constructor for FrontierExploreTests
   *
   *   @param nothing
   *   @return nothing
   */
  FrontierExploreTests() {}
  /**
   *   @brief Default destructor for FrontierExploreTests
   *
   *   @param nothing
   *   @return nothing
   */
  ~FrontierExploreTests() {}
  /**
   *   @brief setup up function to set up the tests with variables or objects
   *
   *   @param nothing
   *   @return nothing
   */
  void SetUp() {}
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
 *   @brief test function to test publisher for velocity
 *
 *
 *   @param FrontierExploreTests, the gtest framework
 *   @param testPubIsOkay, the name of test
 */
TEST_F(FrontierExploreTests, testPubIsOkay) {
  ros::Rate loopRate(10);
  ros::Subscriber velS_ =
      testnh.subscribe("/mobile_base/commands/velocity", 1000,
                       &PublisherTests::vel_Tester, &pubTest);
  loopRate.sleep();
  EXPECT_EQ(1, velS_.getNumPublishers());
}
/**
 *   @brief test function to test /map subscription
 *
 *
 *   @param FrontierExploreTests, the gtest framework
 *   @param testMapIsOkay, the name of test
 */
TEST_F(FrontierExploreTests, testMapIsOkay) {
  ros::Rate loopRate(10);
  ros::Publisher testMap_ =
      testnh.advertise<nav_msgs::OccupancyGrid>("/map", 1000);
  loopRate.sleep();
  EXPECT_EQ(1, testMap_.getNumSubscribers());
}
/**
 *   @brief test function to test updateRobotPose method
 *
 *
 *   @param FrontierExploreTests, the gtest framework
 *   @param testUpdateRobotPose, the name of test
 */
TEST_F(FrontierExploreTests, testUpdateRobotPose) {
  EXPECT_EQ(testBot.updateRobotPose(), true);
}
/**
 *   @brief test function to test rotate method
 *
 *
 *   @param FrontierExploreTests, the gtest framework
 *   @param testRotate, the name of test
 */
TEST_F(FrontierExploreTests, testRotate) { EXPECT_EQ(testBot.rotate(), true); }
