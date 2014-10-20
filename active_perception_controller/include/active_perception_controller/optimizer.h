#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <string.h>
#include <gsl/gsl_randist.h>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <active_perception_controller/sensor_model_map.h>

#define DEBUG 1

namespace optimization
{

class RobotMotionModel
{
public:
    RobotMotionModel();
    ~RobotMotionModel();
    void sample(const geometry_msgs::Twist& vel,
            const geometry_msgs::Pose& pose, double delta_t, size_t n_samples,
            geometry_msgs::PoseArray* samples);

    double alpha_v;
    double alpha_vxy;
    double alpha_vw;
    double alpha_w;
    double alpha_wv;
    double alpha_vg;
    double alpha_wg;
private:
    gsl_rng* rng_;
};

class Optimizer
{
public:
    Optimizer();

private:
    /**
     * Callback to process data coming from the person location particle filter
     */
    void personParticleCloudCallback(
            const sensor_msgs::PointCloudConstPtr& msg);

    /**
     * Callback to process data coming from the robot location particle filter
     */
    void robotParticleCloudCallback(
            const geometry_msgs::PoseArrayConstPtr& msg);
    void robotPoseCallback(
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void robotOdomCallback(const nav_msgs::OdometryConstPtr& msg);

    void navMapMetaDataCallback(const nav_msgs::MapMetaDataConstPtr& msg);

    void optimize();

    ros::NodeHandle nh_;
    ros::Subscriber person_cloud_sub_;
    ros::Subscriber robot_cloud_sub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber robot_odom_sub_;
    ros::Subscriber nav_map_metadata_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher predicted_particles_pub_;
    ros::Publisher cost_map_pub_;
    geometry_msgs::Pose robot_pose_;
    geometry_msgs::Twist robot_vel_;
    geometry_msgs::PoseArray robot_particles_;
    sensor_msgs::PointCloud person_particles_;
    RobotMotionModel rmm_;
    SensorModelMap* smm_;
};
}

#endif
