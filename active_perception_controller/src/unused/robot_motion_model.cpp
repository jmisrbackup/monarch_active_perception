/*
 * robot_motion_model.cpp
 *
 *  Created on: Oct 20, 2014
 *      Author: jmessias
 */

#include <active_perception_controller/robot_motion_model.h>

using namespace ros;
using namespace std;
using namespace optimization;
using namespace geometry_msgs;

RobotMotionModel::RobotMotionModel() :
alpha_v(0.2),
alpha_vxy(0.2),
alpha_vw(0.02),
alpha_w(0.02),
alpha_wv(0.02),
alpha_vg(0.2),
alpha_wg(0.2)
{
    long seed = time (NULL) * getpid();
    rng_ = gsl_rng_alloc (gsl_rng_rand48);
    gsl_rng_set (rng_, seed);
}

void
RobotMotionModel::
sample(const Twist& vel, const Pose& pose, double delta_t, size_t n_samples, PoseArray* samples)
{
    samples->poses.clear();
    //Implements sample_motion_model_velocity (from ProbRob) for omnidirectional robots
    double theta = 2*asin(pose.orientation.z);
    Twist vel_sample(vel);
    double x_std = alpha_v*fabs(vel.linear.x) + alpha_vxy*fabs(vel.linear.y) + alpha_wv*fabs(vel.angular.z);
    double y_std = alpha_v*fabs(vel.linear.y) + alpha_vxy*fabs(vel.linear.x) + alpha_wv*fabs(vel.angular.z);
    double w_std = alpha_w*fabs(vel.angular.z) + alpha_vw*hypot(vel.linear.x,vel.linear.y);
    double g_std = alpha_vg*hypot(vel.linear.x,vel.linear.y) + alpha_wg*fabs(vel.angular.z);
    double phi = atan2(vel_sample.linear.y,vel_sample.linear.x) + theta;
    double sphi = sin(phi);
    double cphi = cos(phi);

    for(size_t i = 0; i < n_samples; i++)
    {
        vel_sample.linear.x += gsl_ran_gaussian(rng_, x_std);
        vel_sample.linear.y += gsl_ran_gaussian(rng_, y_std);
        vel_sample.angular.z += gsl_ran_gaussian(rng_, w_std);
        double gamma = gsl_ran_gaussian(rng_, g_std);
        double linear = hypot(vel_sample.linear.x, vel_sample.linear.y);

        double angular = vel_sample.angular.z;
        Pose sample;
        sample.position.x = pose.position.x - linear/angular*sphi
                            + linear/angular*sin(phi + angular*delta_t);
        sample.position.y = pose.position.y + linear/angular*cphi
                            - linear/angular*cos(phi + angular*delta_t);
        double sample_theta = theta + angular*delta_t + gamma*delta_t;
        sample.orientation.z = sin(sample_theta/2.0);
        sample.orientation.w = cos(sample_theta/2.0);
        samples->poses.push_back(sample);
    }
}

RobotMotionModel::~RobotMotionModel()
{
    gsl_rng_free(rng_);
}

