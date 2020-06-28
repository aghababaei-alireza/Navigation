#ifndef MAP2D_H
#define MAP2D_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Quaternion.h>
#include <navigation_msgs/Vector3.h>
#include <tf/tf.h>


/**
 * @class Map2D
 * @brief Provides a 2D map constructed using OccupancyGrid message and Converts 1D array of map data to 2D array.
 */
class Map2D
{
public:
    /**
     * @brief Default constructor. Sets everythings to zero
     */
    Map2D();

    /**
     * @brief Constructor of the class. Given the map, sets the parameters and calculates the 2D array of map data
     * @param map OccupancyGrid message representing the map
     */
    Map2D(nav_msgs::OccupancyGrid map);

    /**
     * @brief Given the map, sets the parameters and calculates the 2D array of map data
     * @param map OccupancyGrid message representing the map
     */
    void Set(nav_msgs::OccupancyGrid map);

    /**
     * @brief Returns the resolution of the map (meter per pixel)
     * @return Resolution of the map
     */
    double getResolution();

    /**
     * @brief Returns the width of the map (number of pixels)
     * @return Width of the map
     */
    int getWidth();

    /**
     * @brief Returns the height of the map (number of pixels)
     * @return Height of the map
     */
    int getHeight();

    /**
     * @brief Given the orientation of the map in the form of quaternion, find the yaw (rotation about z axis)
     * @param q Orientation of the map in the form of quaternion
     * @return Rotation of the map about z axis (Yaw)
     */
    double getYawFromQuaternion(geometry_msgs::Quaternion q);

    /**
     * @brief Returns the position of lower left point of the map.
     * @return the position of lower left point of the map.
     */
    navigation_msgs::Vector3 getOrigin();
    
    std::vector< std::vector<int> > data; /**<2D array containing the map data. 0 means free, 100 means existing obstacle, -1 means unknown. */

private:
    double resolution; /**<Resolution of the map (meter per pixel) */
    int width; /**<Number of pixels in width */
    int height; /**<Number of pixels in height */
    navigation_msgs::Vector3 origin; /**<The position of lower left point of the map */
};

#endif