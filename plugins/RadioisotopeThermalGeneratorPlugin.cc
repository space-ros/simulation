#include "RadioisotopeThermalGeneratorPlugin.hh"

#include <ignition/transport/Node.hh>
#include "ignition/gazebo/Model.hh"

#include <ignition/msgs/double.pb.h>

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
  ignition::transport::Node node;

  /// \brief Publisher for the radioisotope thermal generator output
public:
  ignition::transport::Node::Publisher nominalPowerPub;
};

//////////////////////////////////////////////////
RadioisotopeThermalGeneratorPlugin::RadioisotopeThermalGeneratorPlugin()
    : dataPtr(std::make_unique<RadioisotopeThermalGeneratorPluginPrivate>())
{
}

//////////////////////////////////////////////////
RadioisotopeThermalGeneratorPlugin::~RadioisotopeThermalGeneratorPlugin() = default;

//////////////////////////////////////////////////
void RadioisotopeThermalGeneratorPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                                   ignition::gazebo::EntityComponentManager &_ecm,
                                                   ignition::gazebo::EventManager &_eventMgr)
{
  // Store the pointer to the model the RTG is under
  auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "Radioisotope Thermal Generator plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->modelName = model.Name(_ecm);

  // Load params
  if (_sdf->HasElement("link_name"))
  {
    this->dataPtr->linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->topicName = "/model/" + this->dataPtr->modelName + "/" + this->dataPtr->linkName + "/radioisotope_thermal_generator_output";
    auto validTopic = ignition::transport::TopicUtils::AsValidTopic(this->dataPtr->topicName);
    if (validTopic.empty())
    {
      ignerr << "Failed to create valid topic [" << this->dataPtr->topicName << "]" << std::endl;
      return;
    }
    // Advertise topic where data will be published
    this->dataPtr->nominalPowerPub = this->dataPtr->node.Advertise<ignition::msgs::Float>(validTopic);
  }
  else
  {
    ignerr << "Radioisotope Thermal Generator plugin should have a <link_name> element. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("nominal_power"))
  {
    this->dataPtr->nominalPower = _sdf->Get<double>("nominal_power");
  }
  else
  {
    ignerr << "Radioisotope Thermal Generator plugin should have a <nominal_power> element. "
           << "Failed to initialize." << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void RadioisotopeThermalGeneratorPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                                    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("RadioisotopeThermalGeneratorPlugin::PostUpdate");
  if (_info.paused)
  {
    return;
  }
  // Publish result
  ignition::msgs::Float msg;
  msg.set_data(this->dataPtr->nominalPower);
  this->dataPtr->nominalPowerPub.Publish(msg);
  igndbg << "Published RTG output: " << this->dataPtr->nominalPower << std::endl;
}

IGNITION_ADD_PLUGIN(RadioisotopeThermalGeneratorPlugin, ignition::gazebo::System,
                    RadioisotopeThermalGeneratorPlugin::ISystemConfigure,
                    RadioisotopeThermalGeneratorPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RadioisotopeThermalGeneratorPlugin, "simulation::RadioisotopeThermalGeneratorPlugin")
