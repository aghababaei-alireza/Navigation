#include <ros/ros.h>
#include <motion_planning/Map2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <navigation_msgs/Vector2.h>
#include <navigation_core/base_global_planner.h>
#include <astar_global_planner/AStar.h>


Map2D map;
bool checkForSpin = false;

void MapCallback(nav_msgs::OccupancyGrid msg){
    map.Set(msg);
    checkForSpin = true;
}

#define USAGE "Usage: \n" \
              "  GlobalPlanner -h\n"\
              "  GlobalPlanner [-n|--namespace <robot_namespace>]"

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

    ros::init(argc, argv, _namespace + "/GlobalPlanner", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_INFO("Node %s Started.", ros::this_node::getName().c_str());

    

    std::string mapTopic;
    private_nh.param("MapTopic", mapTopic, std::string("/map"));

    std::string planningMethod;
    private_nh.param("GlobalPlanningMethod", planningMethod, std::string("astar"));

    ros::Subscriber map_sub = nh.subscribe(mapTopic, 1, MapCallback);

    checkForSpin = false;

    ROS_INFO("Waiting For map data ...");
    while (!checkForSpin)
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    ROS_INFO("Subscribed to %s topic.", mapTopic.c_str());
    ROS_INFO("Initializing global planner ...");

    navigation_msgs::Vector2 gridWorldSize(
        map.getWidth() * map.getResolution(),
        map.getHeight() * map.getResolution()
    );

    navigation_msgs::Vector3 worldBottomLeft(
        map.getOrigin().x,
        map.getOrigin().y,
        0.0
    );

    navigation_core::BaseGlobalPlanner* planner;

    if (planningMethod == "astar")
    {
        planner = new global_planner::AStar(gridWorldSize, map.getResolution()/2.0, worldBottomLeft, map.data, _namespace);
    }
    else
    {
        ROS_ERROR("\"%s\" is not one of global planning algorithms.", planningMethod.c_str());
        return 0;
    }
    
    
    ros::spin();

    delete planner;
    return 0;
}