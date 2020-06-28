#ifndef DWA_H
#define DWA_H

#include <ros/ros.h>
#include <navigation_msgs/LocalPosePlan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>
#include <navigation_core/base_local_planner.h>

namespace local_planner
{
    class DWA : public navigation_core::BaseLocalPlanner{
    public:
        DWA();

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
         * @brief Calculates desired velocity based on the distance to the target
         * @return Desired velocity
         */
        double CalcDesiredVel();

        /**
         * @brief Calculates the distance between two points
         * @param poseA First point
         * @param poseB Second point
         * @return Distance
         */
        double GetDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB);

        /**
         * @brief Calculates the boundaries of allowed velocities
         * @param Vc Current linear velocity
         * @param Wc Current angular velocity
         * @param Vmin [out] Minimum allowed linear velocity
         * @param Vmax [out] Maximum allowed linear velocity
         * @param Wmin [out] Minimum allowed angular velocity
         * @param Wmax [out] Maximum allowed angular velocity
         */
        void GenerateDynamicWindow(double Vc, double Wc, double& Vmin, double& Vmax, double& Wmin, double& Wmax);

        /**
         * @brief Calculates the distance to the nearest obstacle along the predicted arc
         * @param v Linear velocity
         * @param w Angular velocity
         * @return Distance
         */
        double GetMinimumDistanceToObstacle(double v, double w);

        /**
         * @brief Calculates the alignment of the robot with the target direction
         * @param v Linear velocity
         * @param w Angular velocity
         * @return Measure of progress towards the goal location
         */
        double CalcTargetHeading(double v, double w);

        /**
         * @brief Calculates how long it takes to hit the nearest obstacle
         * @param distance The distance to the nearest obstacle along the predicted arc
         * @param v Linear velocity
         * @param w Angular velocity
         * @return Measure of how clear the space in front of the robot is
         */
        double CalcClearance(double distance, double v, double w);

        /**
         * @brief Calculates the linear velocity based on desired velocity
         * @param v predicted velocity
         * @param des_v Desired velocity
         * @return Linear velocity
         */
        double CalcVelocity(double v, double des_v);

        /**
         * @brief Set linear and angular velocity to lublish
         * @param Lx Linear velocity
         * @param Az Angular velocity
         */
        void SetSpeed(double Lx, double Az);

        /**
         * @brief Calculates rotation about z axis (Yaw) from a quaternion
         * @param q Orientation of the robot in form of quaternion
         * @return Yaw
         */
        double getYawFromQuaternion(geometry_msgs::Quaternion q);

        /**
         * @brief Set the parameters of a Pose2D variable
         */
        geometry_msgs::Pose2D SetPose2D(double x, double y, double theta);

        /**
         * @brief Given an angle, finds the distance to the obstacle in that direction based on laser scanner data
         */
        double getLaserDist(double angle);


        double dt; /**<Time interval (sec) */
        
        double dv_a; /**<Linear acceleration of the robot */
        double dv_b; /**<Linear deceleration of the robot */
        double dw_a; /**<Angular acceleration of the robot */
        double dw_b; /**<Angular deceleration of the robot */

        double max_allow_v; /**<Maximum allowable linear velocity specified in datasheet (m/s) */
        double max_allow_w; /**<Maximum allowable angular velocity specified in datasheet (rad/s)*/

        double min_window_v; /**<Minimum linear velocity in dynamic window */
        double max_window_v; /**<Maximum linear velocity in dynamic window */
        double min_window_w; /**<Minimum angular velocity in dynamic window */
        double max_window_w; /**<Maximum angular velocity in dynamic window */

        int v_samples; /**<Number of samples to check for linear velocity in dynamic window */
        int w_samples; /**<Number of samples to check for angular velocity in dynamic window */
        double v_increment; /**<distance between two linear velocity samples in dynamic window */
        double w_increment; /**<distance between two angular velocity samples in dynamic window */

        double alpha; /**<Target heading coefficient of objective function */
        double beta; /**<Clearance coefficient of objective function */
        double gamma; /**<Velocity heading coefficient of objective function */

        double robotWidth; /**<Width of the robot using for collision detection */
        int angle_samples; /**<Number of samples to find the minimum distance to obstacles */
        int line_samples; /**<Number of samples to check the collision along robot width */
        double angle_increment; /**<Angular distance between two points on the arc to find minimum distance to obstacles */
        double line_increment; /**<Linear distance between two points along the robot width */
        double sl; /**<Look_ahead distance of used sensor */

        bool checkPose, checkLaser;
    };
} // namespace local_planner


#endif