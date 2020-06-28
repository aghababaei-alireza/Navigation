#ifndef VFH_H
#define VFH_H

#include <ros/ros.h>
#include <navigation_core/base_local_planner.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <vfh_local_planner/VFH_LocalMap.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <navigation_msgs/LocalPosePlan.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>

namespace local_planner
{
    /**
     * @class VFH
     * @brief Vector Field Histogram algorithm for local planning
     */
    class VFH : public navigation_core::BaseLocalPlanner{
    public:
        /**
         * @brief Default constructor.
         */
        VFH();

        /**
         * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the robot
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool ComputeVelocityCommand(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief Check if goal pose has beem achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief local Planner Service Callback. Given a plan as a request, find velocity commands to move through that plan
         * @param req Request including plan
         * @param resp Response including whether the robot reached the target or not
         * @return True if the robot reached the target, false otherwise
         */
        bool LocalPlanCallback(navigation_msgs::LocalPosePlan::Request& req, navigation_msgs::LocalPosePlan::Response& resp);

        /**
         * @brief Odometry topic callback. Given a message, Updates current position of the robot
         * @param msg Odometry message recieved from topic
         */
        void OdometryCallback(nav_msgs::Odometry msg);

        /**
         * @brief Laser scanner topic callback. Given a message, Updates the laser scanner data
         * @param laser Laser scanner message recieved from topic
         */
        void LaserCallback(sensor_msgs::LaserScan laser);

        /**
         * @brief Creates the polar histogram using certainty grid.
         */
        void Polar_Histogram();

        /**
         * @brief Smoothes the polar histogram by using some kinds of averaging.
         */
        void Smoothed_Polar_Histogram();

        /**
         * @brief Finds the best direction for the robot to move by processing the polar histogram
         */
        int Data_Reduction();

        /**
         * @brief Calculates the velocity command to move to the desired direction
         * @param k_c index of the desired direction (or the direction in degrees [0, 360])
         */
        void Speed_Control(int k_c);

        /**
         * @brief Set the linear and angular velocity for 2D navigation to publish to velocity command topic
         * @param Lx Linear velocity along foreward direction
         * @param Az Angular velocity about z axis
         */
        void SetSpeed(double Lx, double Az);
        
        /**
         * @brief  Given an orientation as quaternion, finds the orientation about z axis (Yaw)
         * @param q Quaternion
         * @return Orientation about z axis (Yaw)
         */
        double getYawFromQuaternion(geometry_msgs::Quaternion q);

        /**
         * @brief Finds the distance between two points
         * @param poseA First point
         * @param poseB Second point
         * @return Distance
         */
        double GetDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB);


        bool checkPose, checkLaser;
        double target_direction; 
        double steer_magnitude;
        double steer_direction;

        double POD[360]; /**<Polar histogram */
        double smoothed_POD[360]; /**<Smoothed polar histogram */

        VFH_LocalMap certainty_grid; /**<Certainty grid representing the existance of the obstacles around the robot */
    };
} // namespace local_planner


#endif