/*
 * Copyright (C) 2024 Robin Baran
 * Copyright (C) 2024 Stevedan Ogochukwu Omodolor Omodia
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
 */

#include "RadioisotopeThermalGeneratorPlugin.hh"
#include "gz_compat.hh"

#include <gz/transport/Node.hh>
#include "gz/sim/Model.hh"

#include <gz/msgs/double.pb.h>
#include <gz/msgs/float.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

using namespace simulation;

/// \brief Private data class for RadioisotopeThermalGeneratorPlugin
class simulation::RadioisotopeThermalGeneratorPluginPrivate
{
  /// \brief Name of the link
public:
  std::string linkName;

  /// \brief Name of the model
public:
  std::string modelName;

  /// \brief Name of the topic where the data will be published
public:
  std::string topicName;

  /// \brief Power output of the radioisotope thermal generator
public:
  double nominalPower;

  /// \brief Ignition communication node
public:
  gz::transport::Node node;

  /// \brief Publisher for the radioisotope thermal generator output
public:
  gz::transport::Node::Publisher nominalPowerPub;
};

//////////////////////////////////////////////////
RadioisotopeThermalGeneratorPlugin::RadioisotopeThermalGeneratorPlugin()
    : dataPtr(std::make_unique<RadioisotopeThermalGeneratorPluginPrivate>())
{
}

//////////////////////////////////////////////////
RadioisotopeThermalGeneratorPlugin::~RadioisotopeThermalGeneratorPlugin() = default;

//////////////////////////////////////////////////
void RadioisotopeThermalGeneratorPlugin::Configure(const gz::sim::Entity &_entity,
                                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                                   gz::sim::EntityComponentManager &_ecm,
                                                   gz::sim::EventManager &_eventMgr)
{
  // Store the pointer to the model the RTG is under
  auto model = gz::sim::Model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "Radioisotope Thermal Generator plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->modelName = model.Name(_ecm);

  // Load params
  if (_sdf->HasElement("link_name"))
  {
    this->dataPtr->linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->topicName = "/model/" + this->dataPtr->modelName + "/" + this->dataPtr->linkName + "/radioisotope_thermal_generator_output";
    auto validTopic = gz::transport::TopicUtils::AsValidTopic(this->dataPtr->topicName);
    if (validTopic.empty())
    {
      gzerr << "Failed to create valid topic [" << this->dataPtr->topicName << "]" << std::endl;
      return;
    }
    // Advertise topic where data will be published
    this->dataPtr->nominalPowerPub = this->dataPtr->node.Advertise<gz::msgs::Float>(validTopic);
  }
  else
  {
    gzerr << "Radioisotope Thermal Generator plugin should have a <link_name> element. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("nominal_power"))
  {
    this->dataPtr->nominalPower = _sdf->Get<double>("nominal_power");
  }
  else
  {
    gzerr << "Radioisotope Thermal Generator plugin should have a <nominal_power> element. "
           << "Failed to initialize." << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void RadioisotopeThermalGeneratorPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                                    const gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("RadioisotopeThermalGeneratorPlugin::PostUpdate");
  if (_info.paused)
  {
    return;
  }
  // Publish result
  gz::msgs::Float msg;
  msg.set_data(this->dataPtr->nominalPower);
  this->dataPtr->nominalPowerPub.Publish(msg);
  gzdbg << "Published RTG output: " << this->dataPtr->nominalPower << std::endl;
}

GZ_ADD_PLUGIN(RadioisotopeThermalGeneratorPlugin, gz::sim::System,
                    RadioisotopeThermalGeneratorPlugin::ISystemConfigure,
                    RadioisotopeThermalGeneratorPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(RadioisotopeThermalGeneratorPlugin, "simulation::RadioisotopeThermalGeneratorPlugin")
