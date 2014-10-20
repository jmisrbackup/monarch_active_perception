/*
 * sensor_model_mask.cpp
 *
 *  Created on: Oct 20, 2014
 *      Author: jmessias
 */

#include <active_perception_controller/sensor_model_map.h>

using namespace ros;
using namespace std;
using namespace cv;
using namespace optimization;
using namespace nav_msgs;

SensorModelMap::
SensorModelMap(const string& map_path,
               const MapMetaData& map_metadata) :
sensor_map_(imread(map_path.c_str(), 1)),
map_metadata_(map_metadata)
{}

void
SensorModelMap::
applySensorModel(const sensor_msgs::PointCloud& particle_set,
                 nav_msgs::OccupancyGrid& map)
{
    map.header.frame_id = "/map";
    map.info = map_metadata_;
    map.data.resize(map.info.height*map.info.width,0);

    for(size_t i = 0; i < particle_set.points.size(); i++)
    {
        size_t px = floor((particle_set.points[i].x - map.info.origin.position.x)
                          /map.info.resolution);
        size_t py = floor((particle_set.points[i].y - map.info.origin.position.y)
                          /map.info.resolution);


        map.data[py*map.info.width + px] = 127;
    }
}
