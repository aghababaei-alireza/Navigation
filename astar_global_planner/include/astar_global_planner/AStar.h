#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <astar_global_planner/Grid.h>
#include <navigation_core/base_global_planner.h>
#include <navigation_msgs/GlobalTarget.h>
#include <navigation_msgs/LocalPosePlan.h>
#include <nav_msgs/Odometry.h>
#include <astar_global_planner/Heap.h>

namespace global_planner
{
    /**
     * @class AStar
     * @brief A global planning algorithm called A*
     */
    class AStar : public navigation_core::BaseGlobalPlanner{
    public:
        /**
         * @brief Constructor of the class.
         * @param _gridWorldSize Size of the map including width and height
         * @param _nodeRad Radius of each node
         * @param _worldBottomLeft Position of the lower left point of the grid
         * @param data 2D vector of integer data representing the occupancy information
         */
        AStar(navigation_msgs::Vector2 _gridWorldSize, double _nodeRad, navigation_msgs::Vector3 _worldBottomLeft, std::vector< std::vector<int> >& data);

        /**
         * @brief Given a start pose and a goal pose in the world, compute an obstacle-free path
         * @param startPos The start position
         * @param targetPos The target position
         * @param plan Found plan which is filled by planner (sent by reference)
         * @return True if a valid plan was found, false otherwise
         */
        bool FindPath(navigation_msgs::Vector3 startPos, navigation_msgs::Vector3 targetPos, std::vector<navigation_msgs::Vector3>& plan);

        /**
         * @brief Reverse the path generated to begin from startPos
         * @param startNode Pointer to first node
         * @param targetNode Ponter to last node
         * @param plan Found plan which is filled by planner (sent by reference)
         */
        void RetracePath(Node* startNode, Node* targetNode, std::vector<navigation_msgs::Vector3>& plan);

        /**
         * @brief Calculate the distance between two nodes
         * @param nodeA First node
         * @param nodeB Second node
         * @return Distance
         */
        double GetDistance(Node* nodeA, Node* nodeB);

        /**
         * @brief Checks if the node exists in the vector
         * @param vect Pointer to vector
         * @param node Pointer to the node
         * @return True if exists, false otherwise
         */
        bool Contain(std::vector<Node*>* vect, Node* node);

        /**
         * @brief Global Planner Service Callback. Given a target position as a request, find an obstacle-free path to that target
         * @param req Request including target position
         * @param resp Response including found path and whether path planning was successfull or not
         * @return True if a valid plan was found, false otherwise
         */
        bool GlobalPlanCallback(navigation_msgs::GlobalTarget::Request& req, navigation_msgs::GlobalTarget::Response& resp);

        /**
         * @brief Odometry topic callback. Given a message, Update current position of the robot
         * @param msg Odometry message recieved from topic
         */
        void OdometryCallback(nav_msgs::Odometry msg);



        navigation_msgs::Vector3 currentPos; /**<Current position of the robot (x, y, z). Updated using subscriber to odom topic */
        Grid grid; /**<Map array */
    };
} // namespace global_planner


#endif