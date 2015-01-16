/*
 * rfid_sensor_model.cpp
 *
 *  Created on: Nov 25, 2014
 *      Author: jescap
 */

#include <active_perception_controller/rfid_sensor_model.h>
#include <active_perception_controller/person_particle_filter.h>
#include <tf/transform_datatypes.h>

using namespace tf;
/** Constructor
  \param image_path Image for positive probability model
  \param resolution Resolution of probability maps
  */
RfidSensorModel::RfidSensorModel(const std::string& image_path, double resolution)
{
    map_resolution_ = resolution;

    prob_map_ = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
}

/** Apply the sensor model to obtain the probability of an observation for a given particle
  \param obs_data Observation
  \param particle Particle
  */
double RfidSensorModel::applySensorModel(SensorData &obs_data, const Particle *particle)
{
    RfidSensorData *rfid_data = (RfidSensorData *)&obs_data;
    PersonParticle *particle_data = (PersonParticle *)particle;

    double det_prob;

    /*
      tf_from_robot -> transformation from robot to global coordinate system
      tf_to_robot -> transformation from global to robot coordinate system
      local_position = tf_to_robot*global_position
      local_position is the position of the particle in the robot coordinate system
      From the local_position, the pixels on the image are computed and the element checked for probability
    */

    Transform robot_tf;
    Vector3 rob_translation(tfScalar(rfid_data->pose_.pose.position.x),tfScalar(rfid_data->pose_.pose.position.y),tfScalar(rfid_data->pose_.pose.position.z));
    Vector3 global_position(particle_data->pose_[0],particle_data->pose_[1],0.0);

    Quaternion rob_rotation;
    quaternionMsgToTF(rfid_data->pose_.pose.orientation, rob_rotation);
    robot_tf.setRotation(rob_rotation);
    robot_tf.setOrigin(rob_translation);

    Transform tf_to_robot = robot_tf.inverse();
    Vector3 local_position = tf_to_robot(global_position);

    int col, row;

    col = (int)(local_position.getX()/map_resolution_ + prob_map_.cols/2.0);
    row = prob_map_.rows - 1 - (int)(local_position.getY()/map_resolution_ + prob_map_.rows/2.0);

    if(col >= 0 && col < prob_map_.cols && row >= 0 && row < prob_map_.rows)
    {
        uchar value = *(prob_map_.data + prob_map_.step*col + row*prob_map_.elemSize());
        det_prob = value/255.0;
    }
    else
    {
        det_prob = 0.0;
    }

    if(rfid_data->rfid_ == true)
        return det_prob; //TODO: should be P(true positive)*det_prob + P(false positive)
    else
        return 1-det_prob; //TODO: should be P(true negative)*(1-det_prob) + P(false negative)
}
