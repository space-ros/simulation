/*
 * Copyright (C) 2024 Robin Baran
 * Copyright (C) 2024 Stevedan Ogochukwu Omodolor Omodia
 * Copyright (C) 2026 Alexey Simonov
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

#include "SolarPanelPlugin.hh"
#include "gz_compat.hh"

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/float.pb.h>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Visual.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/rendering/Events.hh>

#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>
#include <gz/common/Event.hh>

using namespace simulation;

/// \brief Private data class for SolarPanelPlugin
class simulation::SolarPanelPluginPrivate
{
  /// \brief Get visual children of the link
  /// \param[in] _ecm Entity component manager
public:
  std::vector<std::string> GetVisualChildren(
      const gz::sim::EntityComponentManager &_ecm);

  /// \brief Find the rendering scene. Must be called on the render thread.
public:
  bool FindScene();

  /// \brief Ray cast from the sun to the panel and record whether the panel
  /// is the first thing the ray hits. Runs on the render thread.
public:
  void OnPostRender();

  /// \brief Pointer to rendering scene
public:
  gz::rendering::ScenePtr scene{nullptr};

  /// \brief Ray query, created once the scene is available
public:
  gz::rendering::RayQueryPtr rayQuery{nullptr};

  /// \brief Connection to the post-render event
public:
  gz::common::ConnectionPtr postRenderConn;

  /// \brief Protects the data shared between the simulation and render threads
public:
  std::mutex mutex;

  /// \brief Name of the link
public:
  std::string linkName;

  /// \brief Name of the model
public:
  std::string modelName;

  /// \brief Name of the topic where the data will be published
public:
  std::string topicName;

  /// \brief Power output of the solar panel
public:
  double nominalPower{0.0};

  /// \brief Vector of strings containing the scoped names of the visual children
public:
  std::vector<std::string> scopedVisualChildren;

  /// \brief Model interface
public:
  gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Link entity
public:
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};

  /// \brief Gazebo communication node
public:
  gz::transport::Node node;

  /// \brief Publisher for the solar panel output
public:
  gz::transport::Node::Publisher nominalPowerPub;

  /// \brief Ray origin (the sun), written by the simulation thread
public:
  gz::math::Vector3d rayOrigin;

  /// \brief Ray direction (sun to panel), written by the simulation thread
public:
  gz::math::Vector3d rayDirection;

  /// \brief Whether rayOrigin and rayDirection have been set
public:
  bool rayValid{false};

  /// \brief Whether the panel is in the sun's line of sight, written by the
  /// render thread
public:
  bool isInLOS{false};
};

//////////////////////////////////////////////////
SolarPanelPlugin::SolarPanelPlugin()
    : dataPtr(std::make_unique<SolarPanelPluginPrivate>())
{
}

//////////////////////////////////////////////////
SolarPanelPlugin::~SolarPanelPlugin() = default;

//////////////////////////////////////////////////
void SolarPanelPlugin::Configure(const gz::sim::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 gz::sim::EntityComponentManager &_ecm,
                                 gz::sim::EventManager &_eventMgr)
{
  // Store the pointer to the model the solar panel is under
  auto model = gz::sim::Model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "Solar panel plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->model = model;
  this->dataPtr->modelName = model.Name(_ecm);

  // Load params
  if (_sdf->HasElement("link_name"))
  {
    this->dataPtr->linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->topicName = "/model/" + this->dataPtr->modelName + "/" + this->dataPtr->linkName + "/solar_panel_output";
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
    gzerr << "Solar panel plugin should have a <link_name> element. "
          << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("nominal_power"))
  {
    this->dataPtr->nominalPower = _sdf->Get<double>("nominal_power");
  }
  else
  {
    gzerr << "Solar panel plugin should have a <nominal_power> element. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // The line-of-sight check needs the rendering scene, which may only be used
  // from the render thread. PostUpdate runs on parallel worker threads, so the
  // ray cast is done from the post-render event instead.
  this->dataPtr->postRenderConn = _eventMgr.Connect<gz::sim::events::PostRender>(
      std::bind(&SolarPanelPluginPrivate::OnPostRender, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void SolarPanelPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                  const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }

  // Get sun entity
  gz::sim::Entity sunEntity{gz::sim::kNullEntity};
  gz::math::Pose3d sunPose;
  _ecm.Each<gz::sim::components::Name, gz::sim::components::Pose>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Name *_name,
          const gz::sim::components::Pose *_pose) -> bool
      {
        if (_name->Data() == "sun")
        {
          sunEntity = _entity;
          sunPose = _pose->Data();
          return false; // Stop iteration
        }
        return true;
      });

  if (sunEntity == gz::sim::kNullEntity)
  {
    gzerr << "Sun entity not found" << std::endl;
    return;
  }

  // Check if sun entity is of type "light" and has a "direction" element
  const auto *lightComp = _ecm.Component<gz::sim::components::Light>(sunEntity);
  gz::math::Vector3d direction;
  if (lightComp)
  {
    const auto &light = lightComp->Data();
    direction = light.Direction();
  }
  else
  {
    gzerr << "Sun entity is not a light!" << std::endl;
    return;
  }

  // Rotate sun direction according to sun pose orientation
  gz::math::Vector3d sunDirection = sunPose.Rot().RotateVector(direction);

  if (this->dataPtr->linkEntity == gz::sim::kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
    if (this->dataPtr->linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "Link [" << this->dataPtr->linkName << "] not found" << std::endl;
      return;
    }
  }

  gz::math::Pose3d linkPose = gz::sim::worldPose(this->dataPtr->linkEntity, _ecm);

  // Hand the ray from the sun to the panel over to the render thread and take
  // back its latest line-of-sight result.
  bool isInLOS = false;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (this->dataPtr->scopedVisualChildren.empty())
    {
      this->dataPtr->scopedVisualChildren = this->dataPtr->GetVisualChildren(_ecm);
    }
    this->dataPtr->rayOrigin = sunPose.Pos();
    this->dataPtr->rayDirection = linkPose.Pos() - sunPose.Pos();
    this->dataPtr->rayValid = true;
    isInLOS = this->dataPtr->isInLOS;
  }

  // Compute current power output
  float currentPower = 0.0F;

  // Compute the angle between the link normal and sun direction
  // Calculate dot product
  gz::math::Vector3d linkNormal = linkPose.Rot().RotateVector(gz::math::Vector3d::UnitZ);
  float dotProduct = linkNormal.Dot(-sunDirection); // Negate sunDirection because it points from sun to scene

  // Solar panel will not receive any power if angle is more than 90deg (sun rays hitting horizontally or below)
  if ((dotProduct > 0.0F) && (isInLOS))
  {
    // Calculate magnitudes
    float magnitude1 = linkNormal.Length();
    float magnitude2 = sunDirection.Length();
    // Calculate cosine of the angle
    float cosAngle;
    if (gz::math::equal(magnitude1, 0.0F) ||
        gz::math::equal(magnitude2, 0.0F))
    {
      cosAngle = 1.0F;
    }
    else
    {
      cosAngle = dotProduct / (magnitude1 * magnitude2);
    }

    // Compute the effective area factor (cosine of the angle)
    float effectiveAreaFactor = std::max(0.0F, cosAngle);

    // Compute current power based on the angle
    currentPower = this->dataPtr->nominalPower * effectiveAreaFactor;
  }

  // Publish result
  gz::msgs::Float msg;
  msg.set_data(currentPower);
  this->dataPtr->nominalPowerPub.Publish(msg);

  gzdbg << "Solar Panel Plugin:: Current power output: " << currentPower << " watts" << std::endl;
  gzdbg << "Solar Panel Plugin:: In line of sight: " << (isInLOS ? "Yes" : "No") << std::endl;
}

//////////////////////////////////////////////////
void SolarPanelPluginPrivate::OnPostRender()
{
  if (!this->scene)
  {
    if (!this->FindScene())
    {
      return;
    }
  }

  if (!this->rayQuery)
  {
    this->rayQuery = this->scene->CreateRayQuery();
    if (!this->rayQuery)
    {
      gzerr << "Failed to create RayQuery" << std::endl;
      return;
    }
  }

  gz::math::Vector3d origin;
  gz::math::Vector3d direction;
  std::vector<std::string> visualChildren;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->rayValid)
    {
      return;
    }
    origin = this->rayOrigin;
    direction = this->rayDirection;
    visualChildren = this->scopedVisualChildren;
  }

  // Perform ray cast from the sun to the panel: the panel is lit if it is
  // the first thing the ray hits.
  this->rayQuery->SetOrigin(origin);
  this->rayQuery->SetDirection(direction);
  auto result = this->rayQuery->ClosestPoint();

  bool inLOS = false;
  if (result)
  {
    gz::rendering::NodePtr node = this->scene->NodeById(result.objectId);
    if (node)
    {
      const std::string objectName = node->Name();
      inLOS = std::any_of(visualChildren.begin(), visualChildren.end(),
                          [&](const std::string &_elem) { return _elem == objectName; });
    }
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  this->isInLOS = inLOS;
}

//////////////////////////////////////////////////
bool SolarPanelPluginPrivate::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return false;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzwarn << "More than one engine is available. "
           << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
          << "]. Solar panel plugin won't work." << std::endl;
    return false;
  }

  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return false;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return false;
  }

  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
          << "Using scene [" << scenePtr->Name() << "]" << std::endl;
  }

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return false;
  }

  this->scene = scenePtr;
  return true;
}

//////////////////////////////////////////////////
std::vector<std::string> SolarPanelPluginPrivate::GetVisualChildren(
    const gz::sim::EntityComponentManager &_ecm)
{
  // Build the prefix for the scoped name
  std::string scopedPrefix = this->modelName + "::" + this->linkName + "::";

  // Find all visual entities that are children of this link
  std::vector<std::string> visualChildren;
  _ecm.Each<gz::sim::components::Visual, gz::sim::components::Name, gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &,
          const gz::sim::components::Visual *,
          const gz::sim::components::Name *_name,
          const gz::sim::components::ParentEntity *_parent) -> bool
      {
        if (_parent->Data() == this->linkEntity)
        {
          visualChildren.push_back(scopedPrefix + _name->Data());
        }
        return true;
      });

  return visualChildren;
}

GZ_ADD_PLUGIN(SolarPanelPlugin, gz::sim::System,
              SolarPanelPlugin::ISystemConfigure,
              SolarPanelPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(SolarPanelPlugin, "simulation::SolarPanelPlugin")
