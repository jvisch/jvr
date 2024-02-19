#include "jvr_sim/UltrasonicSensor.hpp"

#include <gz/common/Console.hh>

namespace jvr
{

    namespace sim
    {
        UltrasonicSensor::UltrasonicSensor(/* args */)
        {
        }

        UltrasonicSensor::~UltrasonicSensor()
        {
        }

        /*virtual*/ bool UltrasonicSensor::Update(const std::chrono::steady_clock::duration &_now) /*override*/
        {
            gzdbg << "UltrasonicSensor::Update TODO" << std::endl;
            return true;
        }
    }
}