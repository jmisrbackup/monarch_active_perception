#include <ros/ros.h>

#include <active_perception_controller/motion_planner.h>

using namespace ros;
using namespace std;

int main ( int argc, char** argv )
{
    init ( argc, argv, "optimization_node" );
    
    MotionPlanner mp;
    
    spin();
    
    return 0;
}
