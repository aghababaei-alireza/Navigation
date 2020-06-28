#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <navigation_msgs/Vector3.h>

namespace global_planner
{
    /**
     * @class Node
     * @brief Describe each pixel in the map for global planning
     */
    class Node{
    public:
        /**
         * @brief Default constructor. Sets every things to zero
         */
        Node();

        /**
         * @brief Constructor of the class.
         * @param _walkable True if this pixel is free, false there is an obstacle
         * @param _worldPos The position of this pixel in the world frame
         * @param _gridX The index of the pixel inside the map data array along x axis (2nd index)
         * @param _gridY The index of the pixel inside the map data array along y axis (1st index)
         */
        Node(bool _walkable, navigation_msgs::Vector3 _worldPos, int _gridX, int _gridY);

        /**
         * @brief Prints some information about this pixel
         * @return String including some information about this pixel
         */
        std::string Print();

        /**
         * @brief Checks if this node equals to the given node
         * @return True if equals, false otherwise
         */
        bool operator == (Node node);

        /**
         * @brief Checks if this node equals to the given node
         * @return True if equals, false otherwise
         */
        bool operator == (Node* node);

        /**
         * @brief Checks if this node doesn't equal to the given node
         * @return False if equals, true otherwise
         */
        bool operator != (Node node);

        /**
         * @brief Checks if this node doesn't equal to the given node
         * @return False if equals, true otherwise
         */
        bool operator != (Node* node);

        /**
         * @brief Calculate total cost of path through this node (= gCost + hCost)
         * @return Total cost of the path
         */
        double fCost();

        /**
         * @brief Compares two items of type Node
         * @param item Item to compare to this item
         * @return 1 if this item has higher priority, -1 if passed item has higher priority, 0 if both have same priority
         */
        int CompareTo(Node* item);

        bool walkable; /**<True if this node is free false if occupied with an obstacle. */      
        navigation_msgs::Vector3 worldPosition; /**<Position of the node (x, y, z) (type double) */        
        int gridX; /**<index of the node in the grid array (along x direction - 2nd index) */        
        int gridY; /**<index of the node in the grid array (along y direction - 1st index) */        
        double gCost; /**<Cost of the path from startNode to this node */
        double hCost; /**<Cost of the path from this node to TargetNode */
        Node* parent; /**<Pointer to parent of this node */
        int HeapIndex; /**<Index of the item in heap list */
    };
} // namespace global_planner


#endif