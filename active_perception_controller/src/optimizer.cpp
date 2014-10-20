#include <active_perception_controller/optimizer.h>

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace optimization;

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
smm_ (NULL)
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
    if(smm_ != NULL)
    {
        person_particles_ = *msg;
        OccupancyGrid map;
        smm_->applySensorModel(person_particles_,map);
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
    smm_ = new SensorModelMap(path+"/config/sensormodel.png", *msg);
}

void
Optimizer::
optimize()
{
}
