/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/05/05
* 		- File created.
*
* Description:
*	Node to statet the altered move_base, move_base_smc.
* 
***********************************************************************************/

#include <rose_move_base_smc/move_base_smc_node.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rose_move_base_smc");
	ros::NodeHandle n;

	MoveBaseSMC move_base_smc(n, "rose_move_base_smc");

	ros::spin();

	return 0;
}
