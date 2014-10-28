#include <active_perception_controller/optimizer.h>

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace optimization;

Optimizer::
Optimizer() :
person_cloud_sub_ ( nh_.subscribe ( "person_particle_cloud", 10, &Optimizer::personParticleCloudCallback, this ) ),
robot_cloud_sub_ ( nh_.subscribe ( "particle_cloud", 10, &Optimizer::robotParticleCloudCallback, this ) ),
robot_pose_sub_ ( nh_.subscribe ( "amcl_pose", 10, &Optimizer::robotPoseCallback, this ) ),
robot_odom_sub_( nh_.subscribe ( "odom", 10, &Optimizer::robotOdomCallback, this ) ),
nav_map_metadata_sub_ ( nh_.subscribe ( "map_metadata", 10, &Optimizer::navMapMetaDataCallback, this ) ),
cmd_vel_pub_ ( nh_.advertise<Twist> ( "cmd_vel", 10 ) ),
predicted_particles_pub_ ( nh_.advertise<PoseArray> ( "predicted_particle_cloud", 10 ) ),
cost_map_pub_ ( nh_.advertise<OccupancyGrid> ( "optimizer_cost_map", 10 ) ),
rsm_ (NULL)
{
    double alpha_v, alpha_vxy, alpha_vw, alpha_wv, alpha_w, alpha_vg, alpha_wg;
    if(nh_.getParam("alpha_v", alpha_v)) rmm_.alpha_v = alpha_v;
    if(nh_.getParam("alpha_vxy", alpha_vxy)) rmm_.alpha_vxy = alpha_vxy;
    if(nh_.getParam("alpha_vw", alpha_vw)) rmm_.alpha_vw = alpha_vw;
    if(nh_.getParam("alpha_w", alpha_w)) rmm_.alpha_w = alpha_w;
    if(nh_.getParam("alpha_wv", alpha_wv)) rmm_.alpha_wv = alpha_wv;
    if(nh_.getParam("alpha_vg", alpha_vg)) rmm_.alpha_vg = alpha_vg;
    if(nh_.getParam("alpha_wg", alpha_wg)) rmm_.alpha_wg = alpha_wg;
    robot_particles_.header.frame_id = "/map";
}

void
Optimizer::
personParticleCloudCallback(const PointCloudConstPtr& msg)
{
    if(rsm_ != NULL)
    {
        person_particles_ = *msg;
        OccupancyGrid map;
        rsm_->applySensorModel(person_particles_,map);
        cost_map_pub_.publish(map);
    }
}

void
Optimizer::
robotParticleCloudCallback(const PoseArrayConstPtr& msg)
{
    robot_particles_ = *msg;
}

void
Optimizer::
robotPoseCallback(const PoseWithCovarianceStampedConstPtr& msg)
{
    robot_pose_ = msg->pose.pose;
}

void
Optimizer::
robotOdomCallback(const OdometryConstPtr& msg)
{
    robot_vel_ = msg->twist.twist;
#ifdef DEBUG
    static long i = 0;
    if(i%10 == 0){
        rmm_.sample(robot_vel_,
                    robot_pose_,
                    1.0,
                    100,
                    &robot_particles_);
        predicted_particles_pub_.publish(robot_particles_);
    }
    i++;
#endif    
}

void
Optimizer::
navMapMetaDataCallback(const MapMetaDataConstPtr& msg)
{
    string path = ros::package::getPath("active_perception_controller");
    rsm_ = new RobotSensorModel(path+"/config/sensormodel.png", *msg);
}

void
Optimizer::
optimize()
{
}
