#include <active_perception_controller/motion_planner.h>

using namespace std;
using namespace ros;
using namespace nav_msgs;
using namespace ompl::base;

OccupancyGridSampler::OccupancyGridSampler
(const OccupancyGrid& map,
 const StateSpace* ss) :
StateSampler(ss)
{
    //Pre-computing the free cells from which we can sample
    for(size_t i = 0; i < map.info.width; i++)
        for(size_t j = 0; j < map.info.height; j++)
            if(map.data[j*map.info.width + i] == 0)
                navmap_free_idx_.push_back(std::make_pair(i,j));
    navmap_metadata_ = map.info;
    rng_.setSeed(time(NULL));
}

void
OccupancyGridSampler::sampleUniform
(State* state)
{
    int r = rng_.uniformInt(0,navmap_free_idx_.size()-1);
    std::pair<size_t, size_t> sample_idx = navmap_free_idx_[r];

    double x = navmap_metadata_.origin.position.x + navmap_metadata_.resolution * sample_idx.first;
    double y = - (navmap_metadata_.origin.position.y + navmap_metadata_.resolution * (navmap_metadata_.height - sample_idx.second));

    return;
}

void
OccupancyGridSampler::sampleUniformNear
(State* state,
 const State* near,
 double distance)
{
    return;
}

void
OccupancyGridSampler::sampleGaussian
(State* state,
 const State* mean,
 double stdDev)
{
    return;
}


OccupancyGridValidator::OccupancyGridValidator
(const OccupancyGrid& map,
 SpaceInformation* si) :
StateValidityChecker(si)
{
}

MotionPlanner::MotionPlanner() :
target_cloud_sub_ ( nh_.subscribe ( "target_particle_cloud", 10,  &MotionPlanner::targetParticleCloudCallback, this ) ),
robot_pose_sub_ ( nh_.subscribe ( "amcl_pose", 10, &MotionPlanner::robotPoseCallback, this ) ),
ss_ (2), //2-Dimensional state space
si_ ((StateSpacePtr)(& ss_))
{
    GetMap::Request req;
    GetMap::Response resp;
    ROS_INFO("MotionPlanner:: Requesting the map...");
    while(!service::call("static_map", req, resp))
    {
        ROS_WARN("MotionPlanner:: Request for map failed; trying again...");
        Duration d(2.0);
        d.sleep();
    }
    ROS_INFO("MotionPlanner:: Map retrieved");

    sampler_ = new OccupancyGridSampler(resp.map, &ss_);
    validator_ = new OccupancyGridValidator(resp.map, &si_);
}

void
MotionPlanner::
targetParticleCloudCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
}

void
MotionPlanner::
robotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
}

