/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/07/21
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/
#include "rose_pose_explorer/pose_explorer_node.hpp"

int main( int argc, char **argv )
{

// Set up ROS.
    ros::init(argc, argv, "pose_explorer");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int rate;
    std::string topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("rate", rate, int(10));
    private_node_handle_.param("topic", topic, std::string("pose_explorer"));

    // Create a new ScriptInteractionNode object.
    PoseExplorer* pose_explorer = new PoseExplorer("pose_explorer", n);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    bool stop = false;

    while (n.ok() && !stop)
    {
        ros::spinOnce();
        r.sleep();
    }

    delete pose_explorer;

    return 0;
}