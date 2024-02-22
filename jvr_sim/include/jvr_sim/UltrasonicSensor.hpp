#ifndef JVR_ULTRASONIC_SENSOR_HPP
#define JVR_ULTRASONIC_SENSOR_HPP

#include "gz/sensors/RenderingSensor.hh"

namespace jvr
{
    namespace sim
    {
        class UltrasonicSensor : public gz::sensors::Sensor
        {
        public:
            UltrasonicSensor(/* args */);
            virtual ~UltrasonicSensor();

        public:
            virtual bool Update(const std::chrono::steady_clock::duration &_now) override;

        private:
            /* data */
        };

    }
}

#endif // JVR_ULTRASONIC_SENSOR_HPP