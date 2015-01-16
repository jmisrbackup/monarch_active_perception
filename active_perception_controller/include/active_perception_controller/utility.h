#ifndef UTILITY_H
#define UTILITY_H

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <active_perception_controller/rfid_sensor_model.h>

/** This library implements functions to compute utilities based on information gain.
*/
class Utility
{
public:
    Utility(){};

    Utility(std::string prob_image_path,
            float resolution);

    void setPersonParticles(const std::string& serialized_particles);
    double computeInfoGain(float px,
                           float py,
                           std::vector<double>& prev_weights,
                           std::vector<double>& updated_weights);
private:
    std::vector<Particle*> person_particles_;
    boost::shared_ptr<RfidSensorModel> sensor_model_;
    double last_entropy_;
};

using namespace boost::python;

BOOST_PYTHON_MODULE(ap_utility)
{
    class_<Utility>("Utility")
        .def(init<std::string, float>())
        .def("setPersonParticles", &Utility::setPersonParticles)
        .def("computeInfoGain", &Utility::computeInfoGain);

    class_<std::vector<double> >("VectorOfDoubles")
            .def(vector_indexing_suite<std::vector<double> >() )
        ;
}

#endif
