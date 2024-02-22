#include "jvr_sim/UltrasonicSensor.hpp"

#include <gz/common/Console.hh>

#define PRINT_FUNCTION (gzdbg << __PRETTY_FUNCTION__ << std::endl)

namespace jvr
{

    namespace sim
    {
        UltrasonicSensor::UltrasonicSensor(/* args */)
        {
            PRINT_FUNCTION;
        }

        UltrasonicSensor::~UltrasonicSensor()
        {
        }

        /*virtual*/ bool UltrasonicSensor::Update(const std::chrono::steady_clock::duration &/*_now*/) /*override*/
        {
            PRINT_FUNCTION;
            gzdbg << "UltrasonicSensor::Update TODO" << std::endl;
            return true;
        }
    }
}