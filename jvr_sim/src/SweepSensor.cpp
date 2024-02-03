/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// ---
#include "gz/sim/Model.hh"
#include "gz/sim/Joint.hh"
#include "gz/math/Angle.hh"
#include "gz/math/Rand.hh"
#include "gz/math/Helpers.hh"

// #include "gz/sim/components/JointVelocityCmd.hh"
// #include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
// ---

// Don't forget to include the plugin's header.
#include "jvr_sim/SweepSensor.hpp"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    jvr::sim::SweepSensor,
    gz::sim::System,
    jvr::sim::SweepSensor::ISystemConfigure,
    jvr::sim::SweepSensor::ISystemPreUpdate,
    jvr::sim::SweepSensor::ISystemUpdate,
    jvr::sim::SweepSensor::ISystemPostUpdate,
    jvr::sim::SweepSensor::ISystemReset)

using namespace jvr::sim;

gz::sim::Model model{gz::sim::kNullEntity};

void SweepSensor::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &/*_element*/,
                           gz::sim::EntityComponentManager &/*_ecm*/,
                           gz::sim::EventManager &/*_eventManager*/)
{
  gzdbg << "jvr_sim::SweepSensor::Configure on entity: " << _entity << std::endl;
  model = gz::sim::Model(_entity);  
}

void SweepSensor::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused)
  {
    gzdbg << "jvr_sim::SweepSensor::PreUpdate" << std::endl;
    gzdbg << "--- " << __TIMESTAMP__ << std::endl;

    auto servo = model.JointByName(_ecm, "sweep_sensor_joint");

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

    auto pos = gz::math::Rand::DblUniform(0,  GZ_PI);
    _ecm.SetComponentData<gz::sim::components::JointPositionReset>(servo, {pos});

  }
}

void SweepSensor::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &/*_ecm*/)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "jvr_sim::SweepSensor::Update" << std::endl;
  }
}

void SweepSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &/*_ecm*/)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "jvr_sim::SweepSensor::PostUpdate" << std::endl;
  }
}

void SweepSensor::Reset(const gz::sim::UpdateInfo &/*_info*/,
                       gz::sim::EntityComponentManager &/*_ecm*/)
{
  gzdbg << "jvr_sim::SweepSensor::Reset" << std::endl;
}
