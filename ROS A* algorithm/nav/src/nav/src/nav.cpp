#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <sstream>

// using common std commands
using std::cout;
using std::getline;
using std::cin;
using std::endl;

struct Node {
    int x, y;
    double cost;
    double heuristic;
    Node* parent;

    Node(int x_, int y_, double cost_, double heuristic_, Node* parent_) :
            x(x_), y(y_), cost(cost_), heuristic(heuristic_), parent(parent_) {}

    double totalCost() const {
        return cost + heuristic;
    }
};

// structure to compare nodes 
struct CompareNodes {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->totalCost() > rhs->totalCost();
    }
};

std::vector<Node*> astar(Node* start, Node* goal, const std::vector<std::vector<int>>& map, int width, int height) {
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;
    std::vector<Node*> closedSet;
    openSet.push(start);

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current == goal) {
            std::vector<Node*> path;
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            return path;
        }

        closedSet.push_back(current);

        std::vector<Node*> neighbors; // Populate neighbors vector with adjacent nodes

        for (Node* neighbor : neighbors) {
            if (std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) {
                continue; // Skip already evaluated nodes
            }

            double tentativeCost = current->cost + neighbor->cost;
            if (tentativeCost < neighbor->cost) {
                neighbor->parent = current;
                openSet.push(neighbor);
            }
        }
    }

    return {}; // No path found
}


void callBack(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
	int width = data->info.width;
	int height = data->info.height;
	int index = 0;
	double resolution = data->info.resolution;
	cout << "width: " << width << endl;
        cout << "height: " << height << endl;
	std::vector<std::vector<int>> map(height, std::vector<int>(width));

	for (int i = 0; i < height; i++) 
	{
		for (int j = 0; j < width; j++) 
		{
        	    index = i * width + j;
		    map[i][j] = data->data[index];
        	}
    	}
	
	Node* start = new Node(10, 10, 0, 0, nullptr);
	Node* goal = new Node(50, 50, 0, 0, nullptr);

	std::vector<Node*> path = astar(start, goal, map, width, height);

 	ros::NodeHandle nh;
 	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
	nav_msgs::Path path_msg;
	path_msg.header.frame_id = "map";
	
	for (auto it = path.rbegin(); it != path.rend(); ++it) 
	{
		geometry_msgs::PoseStamped pose;
        	pose.pose.position.x = (*it)->x * resolution;
		pose.pose.position.y = (*it)->y * resolution;
		pose.pose.orientation.w = 1.0;
		path_msg.poses.push_back(pose);
	}

	path_pub.publish(path_msg);


}


int main(int argc, char ** argv)
{
    // initialise ros

	ros::init(argc, argv, "nav");
	ros::NodeHandle n;
	ros::Subscriber map = n.subscribe("map", 1, callBack); // subscribing to the map topic
	ros::spin();
	return 0;

}
