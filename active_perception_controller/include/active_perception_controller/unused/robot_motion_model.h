/*
 * robot_motion_model.h
 *
 *  Created on: Oct 20, 2014
 *      Author: jmessias
 */

#ifndef ROBOT_MOTION_MODEL_H_
#define ROBOT_MOTION_MODEL_H_

#include <gsl/gsl_randist.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

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
}


#endif /* SENSOR_MODEL_MASK_H_ */
