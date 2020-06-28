#include <astar_global_planner/Grid.h>
#include <fstream>
#include <iostream>

namespace global_planner
{
    Grid::Grid(navigation_msgs::Vector2 _gridWorldSize, double _nodeRad, navigation_msgs::Vector3 _worldBottomLeft, std::vector<std::vector<int>>& data)
        : nodeRadius(_nodeRad), gridWorldSize(_gridWorldSize), worldBottomLeft(_worldBottomLeft)
    {
        nodeDiameter = 2.0 * nodeRadius;
        gridSizeX = (int)round(gridWorldSize.x / nodeDiameter);
        gridSizeY = (int)round(gridWorldSize.y / nodeDiameter);

        CreateGrid(data);
    }

    Grid::~Grid(){
        for (int j = 0; j < gridSizeY; j++)
        {
            delete grid[j];
        }
        delete grid;
    }

    void Grid::CreateGrid(std::vector< std::vector<int> >& data){
        //Creating 2D array of nodes
        grid = new Node*[gridSizeY];
        for (int j = 0; j < gridSizeY; j++)
        {
            grid[j] = new Node[gridSizeX];
        }

        for (int j = 0; j < gridSizeY; j++)
        {
            for (int i = 0; i < gridSizeX; i++)
            {
                navigation_msgs::Vector3 worldPoint(
                    worldBottomLeft.x + (i * nodeDiameter) + nodeRadius,
                    worldBottomLeft.y + (j * nodeDiameter) + nodeRadius,
                    worldBottomLeft.z
                );
                bool _walkable = (data[j][i] == 0) ? true : false;

                grid[j][i].walkable = _walkable;
                grid[j][i].worldPosition = worldPoint;
                grid[j][i].gridX = i;
                grid[j][i].gridY = j;
            }
        }
    }

    std::vector<Node*> Grid::GetNeighbours(Node* node){
        std::vector<Node*> neighbours;
        for (int j = -1; j <= 1; j++)
        {
            for (int i = -1; i <= 1; i++)
            {
                if (i == 0 && j == 0) continue;
                int checkX = node->gridX + i;
                int checkY = node->gridY + j;
                if (checkX < 0 || checkX >= gridSizeX || checkY < 0 || checkY >= gridSizeY) continue;

                neighbours.push_back(&grid[checkY][checkX]);
            }
        }
        return neighbours;
    }

    Node* Grid::NodeFromWorldPoint(navigation_msgs::Vector3 worldPosition){
        double percentX = (worldPosition.x - worldBottomLeft.x) / gridWorldSize.x;
        double percentY = (worldPosition.y - worldBottomLeft.y) / gridWorldSize.y;

        int x = (int) round((gridSizeX-1) * percentX);
        int y = (int) round((gridSizeY-1) * percentY);

        return &grid[y][x];
    }

    Node* Grid::NodeFromIndex(int gridX, int gridY){
        return &grid[gridY][gridX];
    }

    void Grid::SavePathToFile(){
        std::ofstream f;
        f.open("map.txt");
        if (f.fail())
        {
            ROS_ERROR("Error opening file.");
            return;
        }
        
        for (int j = gridSizeY-1; j >= 0; j--)
        {
            for (int i = 0; i < gridSizeX; i++)
            {
                if (!grid[j][i].walkable)
                {
                    f.put('0');
                }
                else
                {
                    if (Grid::IsInPath(grid[j][i].worldPosition))
                    {
                        f.put('*');
                    }
                    else
                    {
                        f.put('-');
                    }
                }
            }
            f.put('\n');
        }
        f.close();
    }

    bool Grid::IsInPath(navigation_msgs::Vector3 vect){
        for (std::vector<navigation_msgs::Vector3>::iterator it = path.begin(); it != path.end(); it++)
        {
            if ((*it) == vect)
            {
                return true;
            }
        }
        return false;
    }
} // namespace global_planner
