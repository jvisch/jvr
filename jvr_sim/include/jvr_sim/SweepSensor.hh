#ifndef SWEEPSENSOR_HH_
#define SWEEPSENSOR_HH_

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>

namespace jvr
{
  namespace sim
  {
    class SweepSensor : public gz::sensors::Sensor
    {
    public:
      virtual bool Load(const sdf::Sensor &_sdf) override;
      virtual bool Update(const std::chrono::steady_clock::duration &_now) override;

    private:
      gz::transport::Node::Publisher pub;
    };
  }
}

#endif // SWEEPSENSOR_HH_