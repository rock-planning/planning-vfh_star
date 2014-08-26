#include <iostream>
#include <stdlib.h>
#include <base/Angle.hpp>
#define private public 
#include "vfh_star/TreeSearch.h"

using namespace Eigen;
using namespace vfh_star;

int main()
{
    
    NNLookup l(1.0, 0.05, 3.0/180.0*M_PI);
    
    int max = 200;

    std::vector<TreeNode> nodes;
    
    int iterations = 2000;
    
    nodes.resize(iterations);
    
    for(int i = 0; i < iterations; i++)
    {
	int x = random() % max - max/2;
	int y = random() % max - max/2;
	
	std::cout << "X " << x << " Y " << y << std::endl;

	nodes[i].pose.position.x() = x;
	nodes[i].pose.position.y() = y;
	nodes[i].direction = base::Angle::fromRad(0.1);
	nodes[i].yaw = base::Angle::fromRad(0.1);
	
	l.setNode(&(nodes[i]));
    }

    
    for(int i = 0; i < iterations; i++)
    {
	TreeNode *otherNode = l.getNodeWithinBounds(nodes[i]);
	
	bool correct __attribute__((unused)) = otherNode->getPosition() == nodes[i].getPosition();
	assert(correct);
    }
	
};