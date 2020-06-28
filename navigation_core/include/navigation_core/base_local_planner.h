#ifndef NAVIGATION_CORE_BASE_LOCAL_PLANNER_H
#define NAVIGATION_CORE_BASE_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <navigation_msgs/LocalPosePlan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

namespace navigation_core
{
    /**
     * @class BaseLocalPlanner
     * @brief Provides an interface for local planners used in navigation. All local planners should be derived from this interface.
     */
    class BaseLocalPlanner{
    
    public:
        /**
         * @brief local Planner Service Callback. Given a plan as a request, find velocity commands to move through that plan
         * @param req Request including plan
         * @param resp Response including whether the robot reached the target or not
         * @return True if the robot reached the target, false otherwise
         */
        virtual bool LocalPlanCallback(navigation_msgs::LocalPosePlan::Request& req, navigation_msgs::LocalPosePlan::Response& resp) = 0;

        /**
         * @brief Odometry topic callback. Given a message, Updates current position of the robot
         * @param msg Odometry message recieved from topic
         */
        virtual void OdometryCallback(nav_msgs::Odometry msg) = 0;

        /**
         * @brief Laser scanner topic callback. Given a message, Updates the laser scanner data
         * @param laser Laser scanner message recieved from topic
         */
        virtual void LaserCallback(sensor_msgs::LaserScan laser) = 0;

        /**
         * @brief Shutdown service callback. Stops the local planning process.
         */
        bool ShutdownCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
            isShutDown = true;
            return true;
        }

        /**
         * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the robot
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        virtual bool ComputeVelocityCommand(geometry_msgs::Twist& cmd_vel) = 0;

        /**
         * @brief Check if goal pose has beem achieved by the local planner
         * @return True if achieved, false otherwise
         */
        virtual bool isGoalReached() = 0;
        

    protected:
        /**
         * @brief Default constructor. 
         */
        BaseLocalPlanner() {
            private_nh = ros::NodeHandle("~");
            private_nh.param("Local_Service_name", local_service_name, std::string("/LocalPlannerService"));
            private_nh.param("Odometry_Topic_name", odom_topic_name, std::string("/odom"));
            private_nh.param("Laser_Topic_name", laser_topic_name, std::string("/scan"));
            private_nh.param("Velocity_Topic_name", velocity_topic_name, std::string("/cmd_vel"));

            shutdownSrv = nh.advertiseService("/ShutDownLocalPlanning", &BaseLocalPlanner::ShutdownCallback, this);
        }

        std::string local_service_name; /**<Name of Local planner service */
        std::string odom_topic_name; /**<Name of Odometry topic */
        std::string laser_topic_name; /**<Name of Laser Scan topic */
        std::string velocity_topic_name; /**<Name of velocity command topic */

        bool isShutDown; /**<when it changes to true, indicates that the process have to be shut down */

        ros::NodeHandle nh; /**< Node Handle */
        ros::NodeHandle private_nh; /**< private node handle to retrieve private parameters*/
        ros::Subscriber odomSub; /**<Subscriber to odom topic to get the current pose of the robot */
        ros::Subscriber laserSub; /**<Subscriber to laser scan topic to get the laser scanner data */
        ros::Publisher velPub; /**<Publisher to velocity command topic to move the robot */
        ros::ServiceServer localSrv; /**<Service server to recieve requests for local planning */
        ros::ServiceServer shutdownSrv; /**<Service server to recieve finding pathfinding path paths for shutting down the process */

        geometry_msgs::Pose currentPos; /**<Current position of the robot. Updated every time a message comes from odometry topic */
        geometry_msgs::Pose goalPos; /**<Target position stored here */
        geometry_msgs::Twist currentVel; /**<Current velocity of the robot */
        geometry_msgs::Twist speed; /**<Speed to publish to velocity command topic */
        sensor_msgs::LaserScan laser; /**<Laser scanner data */
    };
} // namespace navigation_core


#endif