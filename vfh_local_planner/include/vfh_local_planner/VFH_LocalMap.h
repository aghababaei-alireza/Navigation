#ifndef VFH_LOCALMAP_H
#define VFH_LOCALMAP_H

#include <cmath>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#define _RAD2DEG	(180./M_PI)
#define _DEG2RAD	(M_PI/180.)

#define WINDOW_SIZE 33

namespace local_planner
{
    class VFH_LocalMap{
    public:
        /**
         * @brief Default constructor of the class.
         */
        VFH_LocalMap();

        /**
         * @brief Destructor of the class.
         */
        ~VFH_LocalMap();

        /**
         * @brief Transforms the coordinate from the frame positioned on center of the cell to the frame positioned on lower-left corner of the cell
         * @param p Coordinate with respect to the frame positioned on center of the cell
         * @return The percentage of distance to lower-left corner of the cell
         */
        inline double CU (double p) {return (p + cellSize/2) / cellSize; }

        /**
         * @brief Converts the coordinate to the index of window array
         * @param x coordinate of the cell with respect to the frame attached to the robot
         * @return index of the cell in window array
         */
        inline int M2CU (double x) {return (int)(windowSize/2 + x/cellSize); }

        /**
         * @brief Converts the index inside the window array to the coordinate
         * @param x index of the cell inside window array
         * @return Coordinate of the cell with respect to the frame attached to the robot
         */
        inline double CU2M (int x) {return (x - windowSize/2)*cellSize; }

        /**
         * @brief Checks if the specified indices are inside the window or not
         * @param x Index of the cell along x axis
         * @param y Index of the cell along y axis
         * @return True if it is inside the area, false otherwise
         */
        inline bool IsIn (int x, int y) {return (0 <= x && x < windowSize) && (0 <= y && y < windowSize); }
        
        /**
         * @brief if the cell with x and y index is inside the window, Sets its value
         * @param x Index of the cell along x axis
         * @param y Index of the cell along y axis
         * @param value Value to set
         */
        inline void SetPixel (int x, int y, double value) { if (IsIn(x, y)) cells[y][x] = value; }
        
        /**
         * @brief Sets the value of the cells which are placed on a line with specific start and end point
         * @param x1 Index of the cell represents the start point of the line along x axis
         * @param y1 Index of the cell represents the start point of the line along y axis
         * @param x2 Index of the cell represents the end point of the line along x axis
         * @param y2 Index of the cell represents the end point of the line along y axis
         */
        void DrawLine (int x1, int y1, int x2, int y2, double value);

        /**
         * @brief Clears the values of the entire window (all of the cells)
         */
        void Clear ();

        /**
         * @brief Given a laser scanner data, Update the value of the window array
         * @param laser Laser scanner data
         * @param robot_theta Orientation of the robot about z axis
         */
        void UpdateSensorValue (sensor_msgs::LaserScan& laser, double robot_theta);

        

        double cellSize; /**<Size of each cell inside the window around the robot */
        int windowSize; /**<Number of cells in the window around the robot in each direction */
        double cells[WINDOW_SIZE][WINDOW_SIZE]; /**<2D array of cells representing the window around the robot */
    };
} // namespace local_planner

/**
 * @brief Finds the angle between two directions and normilizes it to [-PI, PI]
 * @param ang1 First direction
 * @param ang2 Second direction
 * @return Normilized angle between two directions
 */
inline double DeltaRad (double ang1, double ang2)
{
	double da = ang1 - ang2;
	if (-M_PI < da && da < M_PI) return da;
	else {
		da = fmod (da, 2*M_PI);
		if (M_PI <= da) return da - 2*M_PI;
		else if (da <= -M_PI) return da + 2*M_PI;
		else return da;
	}
	return da;
}

/**
 * @brief Round a double number to the nearest integer
 * @param a Double number
 * @return Integer number
 */
inline int INT_ (const double a)
{
	// return (long)floor (a + 0.5);
	return (0 < a)? (int)(a + 0.5) : (int)(a - 0.5);
}

/**
 * @brief Normalizes angle to [0, 360]
 * @param theta Angle 
 * @return Normalized angle
 */
inline int H_ID (double theta)
{
	double alpha = 1.;
	return (int)(theta + 10*360 + alpha/2.)%360;
}


#endif