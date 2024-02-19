#include <gz/msgs/double.pb.h>

#include <string>
#include <unordered_map>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/SensorFactory.hh>

#include <sdf/Sensor.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include "jvr_sim/UltrasonicSensor.hpp"
#include "jvr_sim/UltrasonicSensorSystem.hpp"

using namespace jvr::sim;

//////////////////////////////////////////////////
void UltrasonicSensorSystem::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        // Get sensor's scoped name without the world
        auto sensorScopedName = gz::sim::removeParentScope(
            gz::sim::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          std::string topic = scopedName(_entity, _ecm) + "/UltrasonicSensor";
          data.SetTopic(topic);
        }

        gz::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<UltrasonicSensor>(data);
        if (nullptr == sensor)
        {
          gzerr << "Failed to create UltrasonicSensor [" << sensorScopedName << "]"
                 << std::endl;
          return false;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<gz::sim::components::Name>(
            _parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity,
            gz::sim::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->entitySensorMap.insert(std::make_pair(_entity,
            std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void UltrasonicSensorSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // for (auto &[entity, sensor] : this->entitySensorMap)
    // {
    //   sensor->NewPosition(gz::sim::worldPose(entity, _ecm).Pos());
    //   sensor->Update(_info.simTime);
    // }
  }

  this->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void UltrasonicSensorSystem::RemoveSensorEntities(
    const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *)->bool
      {
        if (this->entitySensorMap.erase(_entity) == 0)
        {
          gzerr << "Internal error, missing UltrasonicSensor for entity ["
                         << _entity << "]" << std::endl;
        }
        return true;
      });
}

GZ_ADD_PLUGIN(UltrasonicSensorSystem, gz::sim::System,
  UltrasonicSensorSystem::ISystemPreUpdate,
  UltrasonicSensorSystem::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(UltrasonicSensorSystem, "custom::UltrasonicSensorSystem")