/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/08/01
* 		- File created.
*
* Description:
*	Given an stamped pose this will generate the required command velocities to 
*   position the base_link in the odometry frame.
* 
***********************************************************************************/

#include "rose_relative_positioning/relative_positioning_node.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "relative_positioning");
	ros::NodeHandle n;

	RelativePositioning relative_positioning(n, "relative_positioning");

	ros::spin();

	return 0;
}
