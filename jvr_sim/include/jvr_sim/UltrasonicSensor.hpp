#ifndef JVR_ULTRASONIC_SENSOR_HPP
#define JVR_ULTRASONIC_SENSOR_HPP

#include "gz/sensors/RenderingSensor.hh"

namespace jvr
{
    namespace sim
    {
        class UltrasonicSensor : public gz::sensors::RenderingSensor
        {
        public:
            UltrasonicSensor(/* args */);
            virtual ~UltrasonicSensor();

        private:
            /* data */
        };

    }
}

#endif // JVR_ULTRASONIC_SENSOR_HPP