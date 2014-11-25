/*
 * rfid_sensor_model.h
 *
 *  Created on: Nov 25, 2014
 *      Author: jescap
 */

#ifndef RFID_SENSOR_MODEL_H_
#define RFID_SENSOR_MODEL_H_

#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseWithCovariance.h>
#include <string>

#include <active_perception_controller/sensor_model.h>

/**
  Class to represent an observation from an RFID sensor
  */
class RfidSensorData: public SensorData
{
public:
    bool rfid_;                                         ///< RFID measurement
    geometry_msgs::PoseWithCovariance pose_;            ///< Pose of the RFID sensor
};

/** RFID sensor model. The model is loaded from a couple of images that give the probability of observation.
  One image gives the probability of positive observation and another for negative observation. The images
  represent the field of view of the RFID sensor, which is assumed to be at the center.
  */
class RfidSensorModel: public SensorModel
{
public:
    RfidSensorModel(const std::string& pos_image_path, const std::string& neg_image_path, double resolution);

    double applySensorModel(SensorData &obs_data, const Particle *particle);

private:
    double map_resolution_;     ///< Resolution of map in meter/pixel
    cv::Mat map_image_pos_;     ///< Map for positive observation
    cv::Mat map_image_neg_;     ///< Map for negative observation
};

#endif /* RFID_SENSOR_MODEL_H_ */
