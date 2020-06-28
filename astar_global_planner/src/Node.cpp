#include <astar_global_planner/Node.h>

namespace global_planner
{
    Node::Node(){
        walkable = true;
        worldPosition.Reset();
        gridX = 0;
        gridY = 0;
        gCost = 0.0;
        hCost = 0.0;
        parent = NULL;
    }

    Node::Node(bool _walkable, navigation_msgs::Vector3 _worldPos, int _gridX, int _gridY){
        walkable = _walkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }

    double Node::fCost(){
        return gCost + hCost;
    }

    std::string Node::Print(){
        std::string s = "Node: Position(" + std::to_string(worldPosition.x) + ", " + std::to_string(worldPosition.y) + ")\tgridX = " + std::to_string(gridX) + "\tgridY = " + std::to_string(gridY);
        return s;
    }

    bool Node::operator == (Node node){
        if (walkable == node.walkable &&
            worldPosition == node.worldPosition &&
            gridX == node.gridX && gridY == node.gridY)
        {
            return true;
        }
        else
        {
            return false;
        }        
    }

    bool Node::operator == (Node* node){
        return (*this == *node);
    }

    bool Node::operator != (Node node){
        return !(*this == node);
    }

    bool Node::operator != (Node* node){
        return !(*this == *node);
    }

    int Node::CompareTo(Node* item){
        int compare;
        if (fCost() < item->fCost())
            compare = 1;
        else if (fCost() > item->fCost())
            compare = -1;
        else{
            if (hCost < item->hCost)
                compare = 1;
            else if (hCost > item->hCost)
                compare = -1;
            else
                compare = 0;
        }
        return compare;
    }
} // namespace global_planner
