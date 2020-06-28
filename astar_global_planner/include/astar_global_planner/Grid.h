#ifndef GRID_H
#define GRID_H

#include <ros/ros.h>
#include <navigation_msgs/Vector2.h>
#include <navigation_msgs/Vector3.h>
#include <astar_global_planner/Node.h>

namespace global_planner
{
    /**
     * @class Grid
     * @brief Provides an interaface to create a 2D map for navigation and calculate the costs along paths
     */
    class Grid{
    public:
        /**
         * @brief Constructor of the class.
         * @param _gridWorldSize Size of the grid (Width and Height)
         * @param _nodeRad Radius of each node
         * @param _worldBottomLeft Position of the lower left point of the grid
         * @param data 2D vector of integer data representing the occupancy information
         */
        Grid(navigation_msgs::Vector2 _gridWorldSize, double _nodeRad, navigation_msgs::Vector3 _worldBottomLeft, std::vector< std::vector<int> >& data);
        
        /**
         * @brief Destructor of the class. Deletes all of the pointers.
         */
        ~Grid();

        /**
         * @brief Initializes the grid
         * @param data 2D vector of integer data representing the occupancy information
         */
        void CreateGrid(std::vector< std::vector<int> >& data);

        /**
         * @brief Given a node, find its neighbours inside the grid
         * @param node Pointer to the node
         * @return Vector of Pointers to the neighbours
         */
        std::vector<Node*> GetNeighbours(Node* node);
        
        /**
         * @brief Given a world position, find poointer to the corresponding node
         * @param worldPosition World position of the node
         * @return Pointer to the corresponding node
         */
        Node* NodeFromWorldPoint(navigation_msgs::Vector3 worldPosition);

        /**
         * @brief Given x and y indices from grid array, find pointer to the corresponding node
         * @param gridX index along x axis (2nd index)
         * @param gridY index along y axis (1st index)
         * @return Pointer to the corresponding node
         */
        Node* NodeFromIndex(int gridX, int gridY);

        /**
         * @brief Save generated path to a text file
         */
        void SavePathToFile();

        /**
         * @brief Given a position, find whether it is on the generated path or not
         * @param vect Position of the point
         * @return True if it is on the path, false otherwise
         */
        bool IsInPath(navigation_msgs::Vector3 vect);
        
        navigation_msgs::Vector2 gridWorldSize; /**<Size of the grid (type double) */       
        double nodeRadius; /**<Radius of each node(pixel) */        
        double nodeDiameter; /**<Diameter of each node(pixel) */        
        Node** grid; /**<A 2D array of nodes to create the grid */        
        navigation_msgs::Vector3 worldBottomLeft; /**<Position of the lower-left point of the map (x, y, z) (type double) */        
        int gridSizeX; /**<Number of nodes in x direction (2nd index) */        
        int gridSizeY; /**<Number of nodes in y direction (1st index) */        
        std::vector<navigation_msgs::Vector3> path; /**<Generated path will be stored here */
    };
} // namespace global_planner


#endif