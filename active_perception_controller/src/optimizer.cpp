#include <active_perception_controller/optimizer.h>
#include <geometry_msgs/Twist.h>

Optimizer::
Optimizer() :
person_cloud_sub_ ( nh_.subscribe ( "person_location_cloud", 1, &Optimizer::personParticleCloudCallback, this ) ),
robot_cloud_sub_ ( nh_.subscribe ( "particle_cloud", 1, &Optimizer::robotParticleCloudCallback, this ) ),
cmd_vel_pub_ ( nh_.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 ) )
{}

void
Optimizer::
personParticleCloudCallback(const PoseArrayConstPtr& msg)
{

}

void
Optimizer::
robotParticleCloudCallback(const PoseArrayConstPtr& msg)
{
    robot_particles_ = *msg;
}

void
Optimizer::
optimize()
{
    
}
