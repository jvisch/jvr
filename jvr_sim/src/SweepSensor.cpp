#include "jvr_sim/SweepSensor.hh"

#include <gz/common/Console.hh>

using namespace jvr::sim;

bool SweepSensor::Load(const sdf::Sensor &/*_sdf*/)
{
  gzdbg << "jvr_sim::sim::SweepSensor::Load" << std::endl;
  return true;
}

//////////////////////////////////////////////////
bool SweepSensor::Update(const std::chrono::steady_clock::duration &/*_now*/)
{
  gzdbg << "jvr_sim::sim::SweepSensor::Update" << std::endl;
  return false;
}
