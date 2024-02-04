#ifndef JVR_SIM_SWEEP_SENSOR_HPP
#define JVR_SIM_SWEEP_SENSOR_HPP

#include <gz/sim/System.hh>

#include <memory>

namespace jvr
{
  namespace sim
  {
    // Forward declaration; class containing private data of
    // of the SweepSensor class
    class SweepSensorData;

    class SweepSensor : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate,
                        public gz::sim::ISystemUpdate,
                        public gz::sim::ISystemPostUpdate,
                        public gz::sim::ISystemReset
    {
    public:
      SweepSensor();
      virtual ~SweepSensor() override = default;

      void Configure(
          const gz::sim::Entity &_entity,
          const std::shared_ptr<const sdf::Element> &_element,
          gz::sim::EntityComponentManager &_ecm,
          gz::sim::EventManager &_eventManager) override;

      void PreUpdate(const gz::sim::UpdateInfo &_info,
                     gz::sim::EntityComponentManager &_ecm) override;

      void Update(const gz::sim::UpdateInfo &_info,
                  gz::sim::EntityComponentManager &_ecm) override;

      void PostUpdate(const gz::sim::UpdateInfo &_info,
                      const gz::sim::EntityComponentManager &_ecm) override;

      void Reset(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

    private:
      std::unique_ptr<SweepSensorData> data;
    };
  }
}
#endif //  JVR_SIM_SWEEP_SENSOR_HPP
