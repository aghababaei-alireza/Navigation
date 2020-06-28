#include <ros/ros.h>
#include <navigation_core/base_local_planner.h>
#include <vfh_local_planner/VFH.h>
#include <dwa_local_planner/DWA.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LocalPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_INFO("Node %s Started.", ros::this_node::getName().c_str());

    std::string planningMethod;
    private_nh.param("LocalPlanningMethod", planningMethod, std::string("vfh"));

    navigation_core::BaseLocalPlanner* planner;

    if (planningMethod == "vfh")
    {
        planner = new local_planner::VFH();
    }
    else if (planningMethod == "dwa")
    {
        planner = new local_planner::DWA();
    }
    else
    {
        ROS_ERROR("\"%s\" is not one of local planning algorithms.", planningMethod.c_str());
        delete planner;
        return 0;
    }
    ros::spin();
    delete planner;
    return 0;
}
