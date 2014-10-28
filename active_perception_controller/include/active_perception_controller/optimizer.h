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

#include <active_perception_controller/robot_motion_model.h>
#include <active_perception_controller/robot_sensor_model.h>

#define DEBUG 1

namespace optimization
{

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
    RobotSensorModel* rsm_;
};
}

#endif
