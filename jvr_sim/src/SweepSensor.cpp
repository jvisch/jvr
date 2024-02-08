#include <string>
#include <gz/common/Console.hh>

#include <gz/plugin/Register.hh>

// ---
#include "gz/sim/Model.hh"
#include "gz/sim/Joint.hh"
// #include "gz/math/Angle.hh"
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
      // some defaults (make configurable later)
      static const std::string default_joint_name;
      static const double default_start_angle;
      // Data
      gz::sim::Model model;
      gz::sim::Joint servo;
      double lower;
      double upper;
      double velocity;
    };
    /*static*/ const std::string SweepSensorData::default_joint_name = "sweep_sensor_joint";
    /*static*/ const double SweepSensorData::default_start_angle = 0;

    SweepSensor::SweepSensor() : data(std::make_unique<SweepSensorData>())
    {
    }

    void SweepSensor::Configure(const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> & /*_element*/,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager & /*_eventManager*/)
    {
      gzdbg << "jvr_sim::SweepSensor::Configure on entity: " << _entity << std::endl;
      gzdbg << "--- compiled at: " << __TIMESTAMP__ << std::endl;

      this->data->model = gz::sim::Model(_entity);
      auto joint = this->data->model.JointByName(_ecm, SweepSensorData::default_joint_name);
      this->data->servo = gz::sim::Joint(joint);
      // set start position (facing front)
      this->data->servo.ResetPosition(_ecm, {SweepSensorData::default_start_angle});
      // angle limits (lower and upper)
      this->data->lower = this->data->servo.Axis(_ecm).value()[0].Lower();
      this->data->upper = this->data->servo.Axis(_ecm).value()[0].Upper();
      gzdbg << "--- lower: " << this->data->lower << std::endl;
      gzdbg << "--- upper: " << this->data->upper << std::endl;
      
      // Velocity
  
      this->data->velocity = 0.1;
    }

    void SweepSensor::PreUpdate(const gz::sim::UpdateInfo &_info,
                                gz::sim::EntityComponentManager &_ecm)
    {
      if (!_info.paused)
      {
        // gzdbg << "jvr_sim::SweepSensor::PreUpdate" << std::endl;
        // gzdbg << "--- realtime  : " << _info.realTime.count() << " -----" << std::endl;
        // gzdbg << "--- simtime   : " << _info.simTime.count() << " -----" << std::endl;
        // gzdbg << "--- delta     : " << _info.dt.count() << " -----" << std::endl;
        // gzdbg << "--- iterations: " << _info.iterations << " -----" << std::endl;

        auto current_angle = this->data->servo.Position(_ecm).value()[0];
        // gzdbg << "--- angle: " << current_angle << std::endl;
        if((current_angle <= this->data->lower) || (current_angle >= this->data->upper)) {
          this->data->velocity *= -1;
        }

        this->data->servo.SetVelocity(_ecm, {this->data->velocity});
        // auto servo = model.JointByName(_ecm, "sweep_sensor_joint");

        // auto vel = _ecm.Component<gz::sim::components::JointVelocityCmd>(servo);

        // if(!vel) {
        //   gzdbg << "--- NO vel -----------------------" << std::endl;
        //    _ecm.CreateComponent(servo, gz::sim::components::JointVelocityCmd({.1}));
        // } else {
        //   gzdbg << "---  vel FOUND WHOHOOO -----------------------" << std::endl;
        //   *vel = gz::sim::components::JointVelocityCmd({100});
        // }

        // auto pos = _ecm.Component<gz::sim::components::JointPosition>(servo);

        // if(!pos) {
        //   gzdbg << "--- NO pos -----------------------" << std::endl;
        //    _ecm.CreateComponent(servo, gz::sim::components::JointPosition({1}));
        // } else {
        //   gzdbg << "---  pos FOUND WHOHOOO -----------------------" << std::endl;
        //   *pos = gz::sim::components::JointPosition({1});
        // }

        // auto pos = gz::math::Rand::DblUniform(0,  GZ_PI);
        // _ecm.SetComponentData<gz::sim::components::JointPositionReset>(servo, {pos});
      }
    }

    void SweepSensor::Update(const gz::sim::UpdateInfo &_info,
                             gz::sim::EntityComponentManager & /*_ecm*/)
    {
      if (!_info.paused)
      {
        // gzdbg << "jvr_sim::SweepSensor::Update" << std::endl;
        // gzdbg << "--- realtime  : " << _info.realTime.count() << " -----" << std::endl;
        // gzdbg << "--- simtime   : " << _info.simTime.count() << " -----" << std::endl;
        // gzdbg << "--- delta     : " << _info.dt.count() << " -----" << std::endl;
        // gzdbg << "--- iterations: " << _info.iterations << " -----" << std::endl;
      }
    }

    void SweepSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                                 const gz::sim::EntityComponentManager & /*_ecm*/)
    {
      if (!_info.paused)
      {
        // gzdbg << "jvr_sim::SweepSensor::PostUpdate" << std::endl;
        // gzdbg << "--- realtime  : " << _info.realTime.count() << " -----" << std::endl;
        // gzdbg << "--- simtime   : " << _info.simTime.count() << " -----" << std::endl;
        // gzdbg << "--- delta     : " << _info.dt.count() << " -----" << std::endl;
        // gzdbg << "--- iterations: " << _info.iterations << " -----" << std::endl;
      }
    }

    void SweepSensor::Reset(const gz::sim::UpdateInfo & /*_info*/,
                            gz::sim::EntityComponentManager & /*_ecm*/)
    {
      // gzdbg << "jvr_sim::SweepSensor::Reset" << std::endl;
    }
  } // namespace sim
} // namespace jvr