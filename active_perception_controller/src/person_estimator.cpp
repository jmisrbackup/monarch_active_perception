#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/Bool.h"
#include "person_particle_filter.h"

#include <string>

using namespace std;

/**
This class implements a particle filter to estimate the position of a person. Measurements from a mobile RFID reader
on board a robot are used.
*/
class PersonEstimator
{
  protected:

    ros::NodeHandle nh_;
    ros::Publisher person_belief_pub_;   /// To publish person pose belief
    ros::Subscriber robot_pose_sub_; 	/// To get robot position
    ros::Subscriber robot_cloud_sub_; 	/// To get robot position particles
    ros::Subscriber rfid_sub_;           /// To get RFID measurements

    double step_duration_;               /// Step duration in seconds
    int num_particles_;                  /// Number of particles of the filter
    string global_frame_id_;             /// Global frame id

    PersonParticleFilter *person_pf_;        /// Particle filter for person
    geometry_msgs::PoseArray robot_cloud_;   /// Particles for robot pose
    double robot_x_;                         /// Robot position
    double robot_y_;
    double robot_x_cov_;
    double robot_y_cov_;
    bool first_rob_pose_;                    /// First robot pose arrived
    bool new_measure_;                       /// Indicates new measure arrived
    bool rfid_mes_;                          /// RFID measurement
    bool filter_running_;                    /// True when the filter is initialized
    nav_msgs::OccupancyGrid map_;            /// Map

    /// Callback to receive the robot poses
    void robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    /// Callback to receive the robot particles
    void robotCloudReceived(const geometry_msgs::PoseArrayConstPtr& pose_msg);
    /// Callback to receive the RFID measures
    void rfidReceived(const std_msgs::BoolConstPtr& rfid_msg);
    /// Retreive the map
    void requestMap();

  public:
  PersonEstimator();
  ~PersonEstimator();
  double getStepDuration();
  void runIteration();

};

/** Constructor
*/
PersonEstimator::PersonEstimator()
{
    ros::NodeHandle private_nh("~");
    double sigma_person;
    double d_threshold;
    double prob_positive_det;
    double prob_false_det;

    // Read parameters
    private_nh.param("step_duration", step_duration_, 0.2);
    private_nh.param("num_particles", num_particles_, 5000);
    private_nh.param("sigma_person", sigma_person, 1.0);
    private_nh.param("sigma_person", d_threshold, 3.0);
    private_nh.param("sigma_person", prob_positive_det, 0.9);
    private_nh.param("sigma_person", prob_false_det, 0.2);
    private_nh.param("global_frame_id", global_frame_id_, string("map"));

    // Subscribe/advertise topics
    person_belief_pub_ = nh_.advertise<geometry_msgs::PoseArray>("person_pose_cloud", 1);
    robot_pose_sub_ = nh_.subscribe("amcl_pose", 1, &PersonEstimator::robotPoseReceived, this);
    robot_cloud_sub_ = nh_.subscribe("particlecloud", 1, &PersonEstimator::robotCloudReceived, this);
    rfid_sub_ = nh_.subscribe("rfid", 1, &PersonEstimator::rfidReceived, this);

    // Read the map
    requestMap();

    person_pf_ = new PersonParticleFilter(num_particles_, &map_, sigma_person, d_threshold, prob_positive_det, prob_false_det);

    first_rob_pose_ = false;
    new_measure_ = false;
    filter_running_ = false;
}

/// Destructor
PersonEstimator::~PersonEstimator()
{
    delete person_pf_;
}

/// Get the step duration
double PersonEstimator::getStepDuration()
{
    return step_duration_;
}

/** This method runs an iteration of the particle filter. Prediction and update.
*/
void PersonEstimator::runIteration()
{
    bool publish_data = false;

    // Initialize filter once first robot pose is available
    if(!filter_running_ && first_rob_pose_)
    {
        person_pf_->initUniform();
        publish_data = true;
    }
    else if(filter_running_)
    {
        // Predict
        person_pf_->predict(step_duration_);
        // Update if new measures
        if(new_measure_)
        {
            person_pf_->update(rfid_mes_, robot_x_, robot_y_, robot_x_cov_, robot_y_cov_);
            new_measure_ = false;
        }
        else
        {
            // When there is no leacture from RFID, consider a negative measure
            rfid_mes_ = false;
            person_pf_->update(rfid_mes_, robot_x_, robot_y_, robot_x_cov_, robot_y_cov_);
        }
        publish_data = true;
    }

    // Publish belief
    if(publish_data)
    {
        geometry_msgs::PoseArray cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = global_frame_id_;
        cloud_msg.poses.resize(person_pf_->getNumParticles());

        // Retreive particles
        for(int i = 0; i < person_pf_->getNumParticles(); i++)
        {
            PersonParticle* particle =  (PersonParticle *)person_pf_->getParticle(i);

            cloud_msg.poses[i].position.x = particle->pose_[0];
            cloud_msg.poses[i].position.y = particle->pose_[1];
        }

        person_belief_pub_.publish(cloud_msg);
    }
}

/** Callback to receive the robot poses.

*/
void PersonEstimator::robotPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    robot_x_ = pose_msg->pose.pose.position.x;
    robot_y_ = pose_msg->pose.pose.position.y;
    robot_x_cov_ = pose_msg->pose.covariance[0];
    robot_y_cov_ = pose_msg->pose.covariance[7];
}

/** Callback to receive the robot cloud.

*/
void PersonEstimator::robotCloudReceived(const geometry_msgs::PoseArrayConstPtr& pose_msg)
{
    robot_cloud_ = *pose_msg;
}

/** Callback to receive the RFID.

*/
void PersonEstimator::rfidReceived(const std_msgs::BoolConstPtr& rfid_msg)
{
    new_measure_ = true;
    rfid_mes_ = rfid_msg->data;
}

/** Retreive the map

*/
void PersonEstimator::requestMap()
{
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the map...");
    while(!ros::service::call("static_map", req, resp))
    {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }
    ROS_INFO("Map retreived");
    map_ = resp.map;
}

/// Main loop
int main(int argc, char **argv)
{
    ros::init(argc, argv, "person_estimator");

    // Read parameters
    PersonEstimator person_filter;

    double step_duration = person_filter.getStepDuration();
    ros::Rate loop_rate(1.0/step_duration);

    //while (ros::ok())
    //{
        ros::spinOnce();
        person_filter.runIteration();
        loop_rate.sleep();
    //}

    return 0;
}
