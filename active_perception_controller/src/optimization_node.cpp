#include <ros/ros.h>

#include <active_perception_controller/optimizer.h>

using namespace ros;
using namespace std;

int main ( int argc, char** argv )
{
    init ( argc, argv, "optimization_node" );
    
    Optimizer opt;
    
    spin();
    
    return 0;
}
