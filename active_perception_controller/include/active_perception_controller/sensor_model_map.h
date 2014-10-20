/*
 * sensor_model_map.h
 *
 *  Created on: Oct 20, 2014
 *      Author: jmessias
 */

#ifndef SENSOR_MODEL_MAP_H_
#define SENSOR_MODEL_MAP_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>

namespace optimization
{
class SensorModelMap
{
public:
    SensorModelMap(const std::string& map_path,
                   const nav_msgs::MapMetaData& map_metadata);

    void applySensorModel(const sensor_msgs::PointCloud& particle_set,
                          nav_msgs::OccupancyGrid& map);
private:
    cv::SparseMat sensor_map_;
    nav_msgs::MapMetaData map_metadata_;
};
}


#endif /* SENSOR_MODEL_MASK_H_ */
