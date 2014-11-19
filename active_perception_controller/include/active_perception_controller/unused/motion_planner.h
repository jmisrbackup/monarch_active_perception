#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <boost/foreach.hpp>
#include <utility>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#define foreach BOOST_FOREACH

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>

class OccupancyGridSampler : public ompl::base::StateSampler
{
public:
    OccupancyGridSampler(const nav_msgs::OccupancyGrid& map,
                         const ompl::base::StateSpace* ss);

    void sampleUniform(ompl::base::State* state);
    void sampleUniformNear(ompl::base::State* state,
                           const ompl::base::State* near,
                           double distance);
    void sampleGaussian(ompl::base::State* state,
                        const ompl::base::State* mean,
                        double stdDev);
private:
    std::vector<std::pair<size_t, size_t> > navmap_free_idx_;
    nav_msgs::MapMetaData navmap_metadata_;
};

class OccupancyGridValidator : public ompl::base::StateValidityChecker
{
public:
    OccupancyGridValidator(const nav_msgs::OccupancyGrid& map,
                           ompl::base::SpaceInformation* si);

    bool isValid(const ompl::base::State* s) const
    { return false; }
private:
    nav_msgs::OccupancyGrid navmap_;
    nav_msgs::MapMetaData navmap_metadata_;
};

class MotionPlanner
{
public:
    MotionPlanner();

    void targetParticleCloudCallback(
            const sensor_msgs::PointCloudConstPtr& msg);

    void robotPoseCallback(
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_cloud_sub_;
    ros::Subscriber robot_pose_sub_;
    ompl::base::RealVectorStateSpace ss_;
    ompl::base::SpaceInformation si_;

    OccupancyGridSampler* sampler_;
    OccupancyGridValidator* validator_;
};

#endif
