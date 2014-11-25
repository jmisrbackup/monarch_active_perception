/*
 * sensor_model.h
 *
 *  Created on: Nov 25, 2014
 *      Author: jescap
 */

#ifndef SENSOR_MODEL_H_
#define SENSOR_MODEL_H_

class Particle;

/**
  Class to represent a measurement from a sensor
  */
class SensorData
{
};

/**
  Class to represent the probability model of a sensor
  */
class SensorModel
{
public:

    virtual double applySensorModel(SensorData &obs_data, const Particle *particle) = 0;
};


#endif /* SENSOR_MODEL_H_ */
