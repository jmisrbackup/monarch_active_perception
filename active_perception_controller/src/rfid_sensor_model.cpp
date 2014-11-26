/*
 * rfid_sensor_model.cpp
 *
 *  Created on: Nov 25, 2014
 *      Author: jescap
 */

#include <active_perception_controller/rfid_sensor_model.h>
#include <active_perception_controller/person_particle_filter.h>


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

    /*
      3d particle = Rot*2d_vector + translation
      2d_vector = Rot-1(3d_particle - translation_robot)
      From the 2d_vector the pixels on the images are computed and the element checked for probability
      */
    /*
    bool found = false;
    double range = 0;
    for(size_t w = 0, h = 0, total = 0;
        w < (width-1)/2.0 && h < (height-1)/2.0 && !found;
        w++, h++)
    {
        double theta = M_PI/2.0;
        long int ctheta = 0, stheta = 1;
        size_t x = w, y = h;

        for(size_t l = 0, k = 0, step = 0;
            l < 2*(width-2*w)+2*(height-2*h) && total < width*height && !found;
            l+=1, total++)
        {
            if(ctheta == 0)
                step = width-2*w;
            else
                step = height-2*h;
            if( l - k > step-1 )
            {
                k = l;
                theta -= M_PI/2.0;
                ctheta = (long int) round(cos(theta));
                stheta = (long int) round(sin(theta));
            }
            x += ctheta;
            y += stheta;
            if(*(model.data + model.step*y + x*model.elemSize()) > 128)
            {
                found = true;
                ROS_INFO_STREAM("found at " << x << " " << y);
                range = hypot((double) x-(width-1)/2.0, (double) y-(height-1)/2.0);
            }
        }
    }
    */


    return 0.0;
}
