#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Util.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components.hh>

// Don't forget to include the plugin's header.
#include "jvr_sim/SweepSensorSystem.hh"

using namespace jvr::sim;

void SweepSensorSystem::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
  //   gzdbg << "jvr_sim::SweepSensorSystem::PreUpdate" << std::endl;

  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::CustomSensor *_custom,
          const gz::sim::components::ParentEntity *_parent) -> bool
      {
        gzdbg << "loading custom sensor (entity: " << _entity << ")" << std::endl;
        auto data = _custom->Data();
        auto typeName = gz::sensors::customType(data);
        if ("sweepsensor" != typeName)
        {
          gzdbg << "Trying to load [sweepsensor] sensor, but got type [" << typeName << "] instead, exiting load." << std::endl;
          return true;
        }
        else
        {
          gzdbg << "Loading custom sensor [" << typeName << "]" << std::endl;
        }

        // name of the sensor
        auto sensorScopedName = gz::sim::scopedName(_entity, _ecm, "::", false);
        gzdbg << "sensorName --" << sensorScopedName << std::endl;
        auto sensorName = gz::sim::removeParentScope(sensorScopedName, "::");
        gzdbg << "sensorName" << sensorName << "--" << std::endl;
        data.SetName(sensorName);

        // Default to scoped name as topic
        if (data.Topic().empty())
        {
          gzdbg << "Topic is empty, set to default" << std::endl;
          auto topic = scopedName(_entity, _ecm) + "/sweepsensor";
          
          data.SetTopic(topic);
        }
        gzdbg << "topic " << data.Topic() << std::endl;

        gz::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<jvr::sim::SweepSensor>(data);
        if (nullptr == sensor)
        {
          gzerr << "Failed to create sweepsensor [" << sensorName << "]"
                << std::endl;
          return false;
        }
        else
        {
          gzdbg << "Sweepsensor [" << sensorName << "] created" << std::endl;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<gz::sim::components::Name>(_parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity, gz::sim::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

void SweepSensorSystem::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager & /*_ecm*/)
{
  // if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "jvr_sim::FullSystem::PostUpdate" << std::endl;
  }
}

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    SweepSensorSystem,
    gz::sim::System,
    SweepSensorSystem::ISystemPreUpdate,
    SweepSensorSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(SweepSensorSystem, "jvr::sim::OdometerSystem")