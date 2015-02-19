/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/09/15
* 		- File created.
*
* Description:
*	Unit tests for the footprint collision checker , http://www.ibm.com/developerworks/aix/library/au-googletestingframework.html
* 
***********************************************************************************/

// Bring in my package's API, which is what I'm testing
#include <geometry_msgs/PoseStamped.h>
#include "rose_footprint_collision_checker/footprint_collision_checker.hpp"
// Bring in gtest
#include <gtest/gtest.h>

bool CB_binarySearchUntil(const Trajectory::iterator& begin, const Trajectory::iterator&end)
{
	Trajectory::iterator iter = begin;
	while(iter != end)
	{
		// Check if any is true
		if(iter->pose.position.x == 1.0)
			return false;
		std::next(iter);
	}
	return true;
}

TEST(footprint_collision_checker, binarySearchUntil)
{

	ros::NodeHandle n("~");
	FootprintCollisionChecker FFC(n);

	geometry_msgs::PoseStamped true_pose;
	geometry_msgs::PoseStamped false_pose;
	true_pose.pose.position.x 	= 1.0;
	false_pose.pose.position.x 	= 0.0;

	std::vector<geometry_msgs::PoseStamped> pose_list;
	
	pose_list.push_back(true_pose);
	EXPECT_EQ(0, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	// Should return not found aka end aka 1 from begin
	pose_list.push_back(false_pose);
	EXPECT_EQ(1, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	pose_list.push_back(false_pose);
	pose_list.push_back(true_pose);
	EXPECT_EQ(1, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	EXPECT_EQ(3, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	EXPECT_EQ(3, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	EXPECT_EQ(4, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	EXPECT_EQ(0, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	pose_list.push_back(true_pose);
	EXPECT_EQ(0, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	EXPECT_EQ(2, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	pose_list.push_back(false_pose);
	EXPECT_EQ(3, std::distance(pose_list.begin(), FFC.binarySearchUntil(pose_list.begin(), pose_list.end(), boost::bind(&CB_binarySearchUntil, _1, _2))));
	pose_list.clear();

}


// // Declare a test
// TEST(TestSuite, testCase1)
// {
// 	<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// // Declare another test
// TEST(TestSuite, testCase2)
// {
// 	<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "footprint_collision_checker_utest");
	return RUN_ALL_TESTS();
}
