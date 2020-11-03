#include <ros/ros.h>
#include <navigation_core/base_local_planner.h>
#include <vfh_local_planner/VFH.h>
#include <dwa__local_planner/DWA.h>

#define USAGE "Usage: \n" \
              "  LocalPlanner -h\n"\
              "  LocalPlanner [-n|--namespace <robot_namespace>]"

int main(int argc, char** argv)
{
    std::string _namespace = "";

    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-h"))
        {
            puts(USAGE);
            return 0;
        }
        
        else if (!strcmp(argv[i], "-n") || !strcmp(argv[i], "--namespace"))
        {
            if(++i < argc)
                _namespace = argv[i];
            else
            {
                puts(USAGE);
                return 1;
            }
        }
    }

    ros::init(argc, argv, "LocalPlanner", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_INFO("Node %s Started.", ros::this_node::getName().c_str());

    std::string planningMethod;
    private_nh.param("LocalPlanningMethod", planningMethod, std::string("vfh"));

    navigation_core::BaseLocalPlanner* planner;

    if (planningMethod == "vfh")
    {
        planner = new local_planner::VFH(_namespace);
    }
    else if (planningMethod == "dwa_")
    {
        planner = new local_planner::DWA(_namespace);
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
