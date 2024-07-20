#include <string>
#include <gz/common/Console.hh>

#include <gz/plugin/Register.hh>

// ---
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/math/Angle.hh>

#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
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
      constexpr static double DEFAULT_VELOCITY{GZ_DTOR(60) / 0.3};
      // constexpr static double DEFAULT_VELOCITY{2.0 /*GZ_DTOR(60) / 0.3*/};
      // Every 5 degree a meausurement
      constexpr static double STEP_MOVE{GZ_DTOR(5)};

      // States
      enum class States
      {
        starting,
        moving,
        measuring
      };

      SweepSensorData(const gz::sim::Entity &_entity, gz::sim::EntityComponentManager &_ecm)
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
        gzdbg << "lower : " << this->lower << std::endl;
        gzdbg << "upper : " << this->upper << std::endl;
        gzdbg << "default velocity : " << DEFAULT_VELOCITY << std::endl;
        gzdbg << "step size : " << STEP_MOVE << std::endl;
      }

      SweepSensorData(const SweepSensorData &) = delete;
      SweepSensorData &operator=(const SweepSensorData &) = delete;
      virtual ~SweepSensorData() = default;

      double getVelocity() const
      {
        return this->velocity;
      }

      void setVelocity(gz::sim::EntityComponentManager &_ecm, const double value)
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

      States getState() const
      {
        return this->state;
      }

      void setState(const States new_state)
      {
        this->state = new_state;
      }

      double getTargetPosition() const
      {
        return this->target_position;
      }

      void setTargetPosition(const double new_position)
      {
        auto lower = getLower();
        if (new_position < lower)
        {
          this->target_position = lower;
        }
        else
        {
          upper = getUpper();
          if (new_position > upper)
          {
            this->target_position = upper;
          }
          else
          {
            this->target_position = new_position;
          }
        }
      }

      std::chrono::steady_clock::duration timer;

    private:
      // Data
      gz::sim::Joint servo{gz::sim::kNullEntity};
      double velocity{DEFAULT_VELOCITY * 0.99}; // after many debug session, this seems the solution for moving correctly.
      double lower{0};
      double upper{0};
      States state{States::starting};
      double target_position{0};
    };

    SweepSensor::SweepSensor()
    {
    }

    void SweepSensor::Configure(const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> & /*_element*/,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager & /*_eventManager*/)
    {
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

        auto state = this->data->getState();

        switch (state)
        {
        case SweepSensorData::States::starting:
        {
          const bool event = true;
          if (event)
          {
            // starting exit
            this->data->setTargetPosition(0); // set initial target position
            // state change
            this->data->setState(SweepSensorData::States::moving);
            // moving entry
            // <nothing>
          }
          else
          {
            // starting do
            // <nothing>
          }
        }
        break;
        case SweepSensorData::States::moving:
        {
          const auto velocity = this->data->getVelocity();
          const auto position = this->data->getPosition(_ecm);
          const auto target_position = this->data->getTargetPosition();
          bool event =
              ((velocity > 0) && (position >= target_position)) ||
              ((velocity < 0) && (position <= target_position));
          if (event)
          {
            // moving exit
            // <nothing>
            // switch state
            this->data->setState(SweepSensorData::States::measuring);
            // measuring entry
            // <nothing>
            this->data->timer = _info.simTime;
          }
          else
          {
            // moving do
            this->data->setVelocity(_ecm, velocity);
          }
        }
        break;
        case SweepSensorData::States::measuring:
        {
          const auto past_time = (_info.simTime - this->data->timer);
          using namespace std::literals::chrono_literals;
          const auto interval = 1s;
          bool event = (past_time >= interval);
          if (event)
          {
            // measuring exit
            // send measure message
            // TODO
            // Set new target position
            const auto velocity = this->data->getVelocity();
            const auto position = this->data->getPosition(_ecm);
            if (velocity > 0)
            {
              // moving left
              const auto upper = this->data->getUpper();
              const auto new_position = position + SweepSensorData::STEP_MOVE;
              if (new_position >= upper)
              {
                // change direction
                this->data->setVelocity(_ecm, -1 * velocity);
                // new target
                this->data->setTargetPosition(position - SweepSensorData::STEP_MOVE);
              }
              else
              {
                this->data->setTargetPosition(new_position);
              }
            }
            else
            {
              // moving right
              const auto lower = this->data->getLower();
              const auto new_position = position - SweepSensorData::STEP_MOVE;
              if (new_position <= lower)
              {
                // change direction
                this->data->setVelocity(_ecm, -1 * velocity);
                // new target
                this->data->setTargetPosition(position + SweepSensorData::STEP_MOVE);
              }
              else
              {
                this->data->setTargetPosition(new_position);
              }
            }
            // switch state
            this->data->setState(SweepSensorData::States::moving);
            // moving entry
            // <nothing>
          }
          else
          {
            // measuring do
            // TODO MEASURE
          }
        }
        break;
        default:
          gzerr << "Unknown state '" << static_cast<int>(state) << "'" << std::endl;
        }
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