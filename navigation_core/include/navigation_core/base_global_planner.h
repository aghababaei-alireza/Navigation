#ifndef NAVIGATION_CORE_BASE_GLOBAL_PLANNER_H
#define NAVIGATION_CORE_BASE_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <navigation_msgs/GlobalTarget.h>
#include <navigation_msgs/LocalPosePlan.h>
#include <navigation_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace navigation_core{

    /**
     * @class BaseGlobalPlanner
     * @brief Provides an interface for global planners used in navigation. All global planners should be derived from this interface.
     */
    class BaseGlobalPlanner{

    public:
        /**
         * @brief Given a start pose and a goal pose in the world, compute an obstacle-free path
         * @param startPos The start position
         * @param targetPos The target position
         * @param plan Found plan which is filled by planner (sent by reference)
         * @return True if a valid plan was found, false otherwise
         */
        virtual bool FindPath(navigation_msgs::Vector3 startPos, navigation_msgs::Vector3 targetPos, std::vector<navigation_msgs::Vector3>& plan) = 0;

        /**
         * @brief Global Planner Service Callback. Given a target position as a request, find an obstacle-free path to that target
         * @param req Request including target position
         * @param resp Response including found path and whether path planning was successfull or not
         * @return True if a valid plan was found, false otherwise
         */
        virtual bool GlobalPlanCallback(navigation_msgs::GlobalTarget::Request& req, navigation_msgs::GlobalTarget::Response& resp) = 0;

        /**
         * @brief Odometry topic callback. Given a message, Update current position of the robot
         * @param msg Odometry message recieved from topic
         */
        virtual void OdometryCallback(nav_msgs::Odometry msg) = 0;
        
    protected:
        /**
         * @brief Default constructor. 
         */
        BaseGlobalPlanner() {
            nh.param("Global_Service_name", global_service_name, std::string("/GlobalPlannerService"));
            nh.param("Local_Service_name", local_service_name, std::string("/LocalPlannerService"));
            nh.param("Odometry_Topic_name", odom_topic_name, std::string("/odom"));
        }

        ros::ServiceServer globalSrv; /**<Service server to recieve requests for finding path */
        ros::Subscriber odomSub; /**<Subscriber to odom topic to get the current pose of the robot */
        ros::ServiceClient localClient; /**<Service client to call Local Planner service */
        ros::NodeHandle nh; /**< Node Handle */
        ros::Time _time; /**< Timer object to measure how long the algorithm takes */

        std::string global_service_name; /**<Name of Global planner service */
        std::string local_service_name; /**<Name of Local planner service */
        std::string odom_topic_name; /**<Name of Odometry topic */

        std::string robot_namespace; /**<Namespace of the topics of the robot */
    };
}

#endif