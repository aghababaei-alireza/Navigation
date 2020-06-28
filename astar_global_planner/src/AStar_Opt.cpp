#include <astar_global_planner/AStar.h>

namespace global_planner
{
    AStar::AStar(navigation_msgs::Vector2 _gridWorldSize, double _nodeRad, navigation_msgs::Vector3 _worldBottomLeft, std::vector< std::vector<int> >& data)
        : grid(_gridWorldSize, _nodeRad, _worldBottomLeft, data)
    {
        globalSrv = nh.advertiseService(global_service_name, &AStar::GlobalPlanCallback, this);
        odomSub = nh.subscribe(odom_topic_name, 1, &AStar::OdometryCallback, this);
        localClient = nh.serviceClient<navigation_msgs::LocalPosePlan>(local_service_name);
        ROS_INFO("AStar Initialized successfully.");
    }

    void AStar::OdometryCallback(nav_msgs::Odometry msg){
        currentPos.Set(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        );
    }

    bool AStar::GlobalPlanCallback(navigation_msgs::GlobalTarget::Request& req, navigation_msgs::GlobalTarget::Response& resp){
        ros::spinOnce();

        bool success = FindPath(currentPos, req.targetPos, grid.path);
        resp.success = success;
        resp.plan = grid.path;
        return success;
    }

    bool AStar::FindPath(navigation_msgs::Vector3 startPos, navigation_msgs::Vector3 targetPos, std::vector<navigation_msgs::Vector3>& plan){
        _time = ros::Time::now();
        bool success = false;

        Node* startNode = grid.NodeFromWorldPoint(startPos);
        Node* targetnode = grid.NodeFromWorldPoint(targetPos);

        ROS_INFO("StartNode  => %s", startNode->Print().c_str());
        ROS_INFO("TargetNode => %s", targetnode->Print().c_str());

        Heap openSet(grid.gridSizeX * grid.gridSizeY);
        std::vector<Node*> closedSet;

        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            /////////// Find the node with minimum cost, remove from openSet and add to closedSet //////////////////
            Node* node = openSet.RemoveFirstItem();

            closedSet.push_back(node);

            ////////// Checking if the target reached //////////////
            if (node == targetnode)
            {
                ROS_INFO("Reached the target. Retracing the path.");
                success = true;
                AStar::RetracePath(startNode, targetnode, grid.path);
                return success;
            }

            /////////// Getting the neighbours of the node and calculating their costs //////////////
            std::vector<Node*> neighbours = grid.GetNeighbours(node);
            for (std::vector<Node*>::iterator it = neighbours.begin(); it != neighbours.end(); it++)
            {
                if (!(*it)->walkable || Contain(&closedSet, *it)) continue;

                double newCostToNeighbour = node->gCost + GetDistance(node, *it);
                if (newCostToNeighbour < (*it)->gCost || !openSet.Contains(*it))
                {
                    (*it)->gCost = newCostToNeighbour;
                    (*it)->hCost = GetDistance((*it), targetnode);
                    (*it)->parent = node;
                    if (!openSet.Contains(*it))
                    {
                        openSet.Add(*it);
                    }
                }
            }
        }
        ROS_INFO("Finished FindPath, %s", success ? "Successfully" : "Failed");
        return success;
    }

    void AStar::RetracePath(Node* startNode, Node* targetNode, std::vector<navigation_msgs::Vector3>& plan){
        ros::Duration t = ros::Time::now() - _time;
        ROS_INFO("Find Path completed in %f seconds.", t.toSec());
        _time = ros::Time::now();

        Node* currentNode = targetNode;
        plan.clear();
        while (currentNode != startNode)
        {
            plan.push_back(currentNode->worldPosition);
            currentNode = currentNode->parent;
        }

        std::reverse(plan.begin(), plan.end());

        t = ros::Time::now() - _time;
        ROS_INFO("Path reversed in %.15f seconds.", t.toSec());
        _time = ros::Time::now();

        ROS_INFO("Preliminary Path => Number of Nodes: %d", (int)plan.size());

        //////////// First Reduction /////////////////
        std::vector<navigation_msgs::Vector3> simplified_path;
        navigation_msgs::Vector3 old2, old1, cur;
        old2 = plan[0];
        simplified_path.push_back(old2);
        old1 = plan[1];
        simplified_path.push_back(old1);

        for (std::vector<navigation_msgs::Vector3>::iterator it = plan.begin()+2; it != plan.end(); it++)
        {
            cur = *it;
            if (atan2(old1.y - old2.y, old1.x - old2.x) == atan2(cur.y - old1.y, cur.x - old1.x))
            {
                simplified_path.pop_back();
            }
            simplified_path.push_back(cur);
            old2 = old1;
            old1 = cur;
        }

        t = ros::Time::now() - _time;
        ROS_INFO("First Reduction of path completed in %.15f seconds.", t.toSec());
        _time = ros::Time::now();
        ROS_INFO("Path after firest reduction => Number of Nodes: %d", (int)simplified_path.size());

        /////////////// Second Reduction //////////////////
        plan.clear();
        plan.resize(simplified_path.size());
        plan = simplified_path;
        simplified_path.clear();

        old2 = plan[0];
        simplified_path.push_back(old2);
        old1 = plan[1];
        simplified_path.push_back(old1);

        for (std::vector<navigation_msgs::Vector3>::iterator it = plan.begin()+2; it != plan.end(); it++)
        {
            cur = *it;
            double theta = atan2(cur.y - old2.y, cur.x - old2.x);
            double l_max = sqrt(pow(cur.x - old2.x, 2) + pow(cur.y - old2.y, 2));
            double inc = 2.0e-2;
            bool collision = false;
            for (double l = 0; l < l_max; l += inc)
            {
                navigation_msgs::Vector3 P;
                P.Set(
                    old2.x + l * cos(theta),
                    old2.y + l * sin(theta),
                    old2.z
                );
                Node* n = grid.NodeFromWorldPoint(P);
                if (!n->walkable)
                {
                    collision = true;
                    break;
                }
            }
            if (!collision)
            {
                simplified_path.pop_back();
            }
            simplified_path.push_back(cur);
            old2 = old1;
            old1 = cur;
        }
        
        t = ros::Time::now() - _time;
        ROS_INFO("Second Reduction of path completed in %.15f seconds.", t.toSec());
        _time = ros::Time::now();

        ROS_INFO("Path after second reduction => Number of Nodes: %d", (int)simplified_path.size());

        plan.clear();
        plan.resize(simplified_path.size());
        plan = simplified_path;

        navigation_msgs::LocalPosePlan msg;
        msg.request.posePlan = plan;
        localClient.call(msg);

        //grid.SavePathToFile();
    }

    double AStar::GetDistance(Node* nodeA, Node* nodeB){
        double dist = pow((nodeA->worldPosition.x - nodeB->worldPosition.x), 2) + pow((nodeA->worldPosition.y - nodeB->worldPosition.y), 2) + pow((nodeA->worldPosition.z - nodeB->worldPosition.z), 2); 
        return sqrt(dist);
    }

    bool AStar::Contain(std::vector<Node*>* vect, Node* node){
        for (std::vector<Node*>::iterator it = vect->begin(); it < vect->end(); it++)
        {
            if (node == (*it))
            {
                return true;
            }
        }
        return false;
    }
} // namespace global_planner
