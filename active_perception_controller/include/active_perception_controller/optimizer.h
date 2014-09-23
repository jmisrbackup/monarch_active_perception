#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

using namespace ros;
using namespace std;
using namespace geometry_msgs;

class Optimizer
{
public:
    Optimizer();
    
private:
    /**
     * Callback to process data coming from the person location particle filter
     */
    void personParticleCloudCallback(const PoseArrayConstPtr& msg);
    
    /**
     * Callback to process data coming from the robot location particle filter
     */
    void robotParticleCloudCallback(const PoseArrayConstPtr& msg);
    
    void optimize();
    
    NodeHandle nh_;
    Subscriber person_cloud_sub_;
    Subscriber robot_cloud_sub_;
    Publisher cmd_vel_pub_;
    
    
    PoseArray robot_particles_;
    PoseArray person_particles_;
};