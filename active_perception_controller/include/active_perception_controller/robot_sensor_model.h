/*
 * robot_sensor_model.h
 *
 *  Created on: Oct 20, 2014
 *      Author: jmessias
 */

#ifndef ROBOT_SENSOR_MODEL_H_
#define ROBOT_SENSOR_MODEL_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>

#include <active_perception_controller/utility_map.h>

namespace optimization
{
class RobotSensorModel
{
public:
    RobotSensorModel(const std::string& map_path,
                   const nav_msgs::MapMetaData& map_metadata);

    void applySensorModel(const sensor_msgs::PointCloud& particle_set,
                          nav_msgs::OccupancyGrid& map);
    void applySensorModel(const geometry_msgs::Point32& particle,
                          nav_msgs::OccupancyGrid& map);
private:
    cv::SparseMat sensor_map_;
    nav_msgs::MapMetaData map_metadata_;
};
}


#endif /* SENSOR_MODEL_MASK_H_ */
