#include <string>
#include <gz/common/Console.hh>

#include <gz/plugin/Register.hh>

// ---
#include "gz/sim/Model.hh"
#include "gz/sim/Joint.hh"
#include "gz/math/Angle.hh"
// #include "gz/math/Rand.hh"
// #include "gz/math/Helpers.hh"

#include "gz/sim/components/JointVelocityCmd.hh"
// #include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
// ---

// Don't forget to include the plugin's header.
#include "jvr_sim/SweepSensor.hpp"

GZ_ADD_PLUGIN(
    jvr::sim::SweepSensor,
    gz::sim::System,
    jvr::sim::SweepSensor::ISystemConfigure,
    jvr::sim::SweepSensor::ISystemPreUpdate,
    jvr::sim::SweepSensor::ISystemUpdate,
    jvr::sim::SweepSensor::ISystemPostUpdate,
    jvr::sim::SweepSensor::ISystemReset)

namespace jvr
{

  namespace sim
  {
    class SweepSensorData
    {
    public:
      // Defaults
      // Set velocity to the max (60 deg per 0.3 secs)
      const double DefaultVelocity = GZ_DTOR(60) / 0.3;

      // States
      enum class States
      {
        moving,
        measuring
      };

      SweepSensorData(const gz::sim::Entity &_entity, gz::sim::EntityComponentManager &_ecm)
          : servo(), velocity(DefaultVelocity)
      {
        // get the joint
        auto model = gz::sim::Model(_entity);
        auto joint = model.JointByName(_ecm, "sweep_sensor_joint");
        this->servo = gz::sim::Joint(joint);
        // set start position (facing front)
        this->servo.ResetPosition(_ecm, {0});
        // Get angle limits
        this->lower = this->servo.Axis(_ecm).value()[0].Lower();
        this->upper = this->servo.Axis(_ecm).value()[0].Upper();
      }

      SweepSensorData(const SweepSensorData&) = delete;
      SweepSensorData& operator=(const SweepSensorData&) = delete; 
      virtual ~SweepSensorData() = default;

      double getVelocity() const
      {
        return this->velocity;
      }

      void setVelocity(gz::sim::EntityComponentManager &_ecm, double value)
      {
        this->velocity = value;
        this->servo.SetVelocity(_ecm, {this->velocity});
      }

      double getPosition(const gz::sim::EntityComponentManager &_ecm) const
      {
        return this->servo.Position(_ecm).value()[0];
      }

      double getLower() const
      {
        return this->lower;
      }

      double getUpper() const
      {
        return this->upper;
      }

    private:
      // Data
      gz::sim::Joint servo;
      double velocity;
      double lower;
      double upper;
      // States state;
      // double moveTo;
    };

    SweepSensor::SweepSensor()
    {
    }

    void SweepSensor::Configure(const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> & /*_element*/,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager & /*_eventManager*/)
    {
      gzdbg << "jvr_sim::SweepSensor::Configure on entity: " << _entity << std::endl;
      data = std::make_unique<SweepSensorData>(_entity, _ecm);
    }

    void SweepSensor::PreUpdate(const gz::sim::UpdateInfo &_info,
                                gz::sim::EntityComponentManager &_ecm)
    {
      if (!_info.paused)
      {
        if (_info.dt < std::chrono::steady_clock::duration::zero())
        {
          gzwarn << "Detected jump back in time ["
                 << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
                 << "s]. System may not work properly." << std::endl;
        }

        const auto position = this->data->getPosition(_ecm);
        auto velocity = this->data->getVelocity();
        const auto lower = this->data->getLower();
        const auto upper = this->data->getUpper();
        if (velocity > 0)
        {
          // Moving left
          if (position >= upper)
          {
            velocity *= -1;
          }
        }
        else if (velocity < 0)
        {
          // Moving right
          if (position <= lower)
          {
            velocity *= -1;
          }
        }

        this->data->setVelocity(_ecm, velocity);
      }
    }

    void SweepSensor::Update(const gz::sim::UpdateInfo & /*_info*/,
                             gz::sim::EntityComponentManager & /*_ecm*/)
    {
      // if (!_info.paused)
      // {
      //   // gzdbg << "jvr_sim::SweepSensor::Update" << std::endl;
      //   // gzdbg << "--- realtime  : " << _info.realTime.count() << " -----" << std::endl;
      //   // gzdbg << "--- simtime   : " << _info.simTime.count() << " -----" << std::endl;
      //   // gzdbg << "--- delta     : " << _info.dt.count() << " -----" << std::endl;
      //   // gzdbg << "--- iterations: " << _info.iterations << " -----" << std::endl;
      // }
    }

    void SweepSensor::PostUpdate(const gz::sim::UpdateInfo & /*_info*/,
                                 const gz::sim::EntityComponentManager & /*_ecm*/)
    {
      // if (!_info.paused)
      // {
      //   // gzdbg << "jvr_sim::SweepSensor::PostUpdate" << std::endl;
      //   // gzdbg << "--- realtime  : " << _info.realTime.count() << " -----" << std::endl;
      //   // gzdbg << "--- simtime   : " << _info.simTime.count() << " -----" << std::endl;
      //   // gzdbg << "--- delta     : " << _info.dt.count() << " -----" << std::endl;
      //   // gzdbg << "--- iterations: " << _info.iterations << " -----" << std::endl;
      // }
    }

    void SweepSensor::Reset(const gz::sim::UpdateInfo & /*_info*/,
                            gz::sim::EntityComponentManager & /*_ecm*/)
    {
      // gzdbg << "jvr_sim::SweepSensor::Reset" << std::endl;
    }
  } // namespace sim
} // namespace jvr