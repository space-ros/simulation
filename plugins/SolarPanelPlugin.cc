#include "SolarPanelPlugin.hh"

#include <ignition/msgs/double.pb.h>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/Visual.hh>

#include <ignition/gazebo/EntityComponentManager.hh>
#include "ignition/gazebo/Model.hh"
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/transport/Node.hh>
#include <ignition/physics/Entity.hh>
#include <ignition/common/Event.hh>

using namespace simulation;

/// \brief Private data class for SolarPanelPlugin
class simulation::SolarPanelPluginPrivate
{
  /// \brief Get visual children of the link
  /// \param[in] _ecm Entity component manager
public:
  std::vector<std::string> GetVisualChildren(
      const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Find the scene
public:
  bool FindScene();

  IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
  /// \brief Event that is used to trigger callbacks when the scene
  /// is changed
  /// \param[in] _scene The new scene
public:
  static ignition::common::EventT<void(const ignition::rendering::ScenePtr &)>
      sceneEvent;
  IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

  /// \brief Pointer to rendering scene
  /// \param[in] _scene Rendering scene
public:
  ignition::rendering::ScenePtr scene{nullptr};

  /// \brief Connection to the Manager's scene change event.
public:
  ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
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
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

  /// \brief Link entity
public:
  ignition::gazebo::Entity linkEntity{ignition::gazebo::kNullEntity};

  /// \brief Ignition communication node
public:
  ignition::transport::Node node;

  /// \brief Publisher for the radioisotope thermal generator output
public:
  ignition::transport::Node::Publisher nominalPowerPub;
};

//////////////////////////////////////////////////
SolarPanelPlugin::SolarPanelPlugin()
    : dataPtr(std::make_unique<SolarPanelPluginPrivate>())
{
}

//////////////////////////////////////////////////
SolarPanelPlugin::~SolarPanelPlugin() = default;

//////////////////////////////////////////////////
void SolarPanelPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 ignition::gazebo::EntityComponentManager &_ecm,
                                 ignition::gazebo::EventManager &_eventMgr)
{
  // Store the pointer to the model the solar panel is under
  auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "Solar panel plugin should be attached to a model entity. "
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
    ignerr << "Solar panel plugin should have a <link_name> element. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("nominal_power"))
  {
    this->dataPtr->nominalPower = _sdf->Get<double>("nominal_power");
  }
  else
  {
    ignerr << "Solar panel plugin should have a <nominal_power> element. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->sceneChangeConnection = this->dataPtr->sceneEvent.Connect(std::bind(&SolarPanelPlugin::SetScene, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void SolarPanelPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                  const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }
  if (!this->dataPtr->scene)
  {
    if (!this->dataPtr->FindScene())
    {
      ignwarn << "Rendering scene not available yet" << std::endl;
      return;
    }
  }

  std::shared_ptr<ignition::rendering::RayQuery> rayQuery = this->dataPtr->scene->CreateRayQuery();
  if (!rayQuery)
  {
    ignerr << "Failed to create RayQuery" << std::endl;
    return;
  }

  if (this->dataPtr->scopedVisualChildren.empty())
  {
    this->dataPtr->scopedVisualChildren = this->dataPtr->GetVisualChildren(_ecm);
  }

  // Get sun entity
  ignition::gazebo::Entity sunEntity;
  ignition::math::Pose3d sunPose;
  _ecm.Each<ignition::gazebo::components::Name, ignition::gazebo::components::Pose>(
      [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Name *_name,
          const ignition::gazebo::components::Pose *_pose) -> bool
      {
        if (_name->Data() == "sun")
        {
          sunEntity = _entity;
          sunPose = _pose->Data();
          return false; // Stop iteration
        }
        return true;
      });

  if (sunEntity == ignition::gazebo::v6::kNullEntity)
  {
    ignerr << "Sun entity not found" << std::endl;
    return;
  }

  // Check if sun entity is of type "light" and has a "direction" element
  const auto *lightComp = _ecm.Component<ignition::gazebo::components::Light>(sunEntity);
  ignition::math::Vector3d direction;
  if (lightComp)
  {
    const auto &light = lightComp->Data();
    direction = light.Direction();
  }
  else
  {
    ignerr << "Sun entity is not a light!" << std::endl;
    return;
  }

  // Rotate sun direction according to sun pose orientation
  ignition::math::Vector3d sunDirection = sunPose.Rot().RotateVector(direction);

  if (this->dataPtr->linkEntity == ignition::gazebo::v6::kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  ignition::math::Pose3d linkPose = ignition::gazebo::worldPose(this->dataPtr->linkEntity, _ecm);
  // Perform ray cast from link to sun
  ignition::math::Vector3d start = linkPose.Pos();
  ignition::math::Vector3d end = sunPose.Pos();

  rayQuery->SetOrigin(end);
  rayQuery->SetDirection(start - end);

  // Check if ray intersects with any obstacles
  auto result = rayQuery->ClosestPoint();
  bool isValid = result;

  std::string objectName = "unknown";
  bool isInLOS = false;
  ignition::rendering::NodePtr node = this->dataPtr->scene->NodeById(result.objectId);
  if (node)
  {
    objectName = node->Name();
    if (isValid)
    {
      isInLOS = (any_of(this->dataPtr->scopedVisualChildren.begin(), this->dataPtr->scopedVisualChildren.end(), [&](const std::string &elem)
                        { return elem == objectName; }));
    }
  }

  // Compute current power output
  float currentPower = 0.0F;

  // Compute the angle between the link normal and sun direction
  // Calculate dot product
  ignition::math::Vector3d linkNormal = linkPose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
  float dotProduct = linkNormal.Dot(-sunDirection); // Negate sunDirection because it points from sun to scene

  // Solar panel will not receive any power if angle is more than 90deg (sun rays hitting horizontally or below)
  if ((dotProduct > 0.0F) && (isInLOS))
  {
    // Calculate magnitudes
    float magnitude1 = linkNormal.Length();
    float magnitude2 = sunDirection.Length();
    // Calculate cosine of the angle
    float cosAngle;
    if (ignition::math::equal(magnitude1, 0.0F) ||
        ignition::math::equal(magnitude2, 0.0F))
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
  ignition::msgs::Float msg;
  msg.set_data(currentPower);
  this->dataPtr->nominalPowerPub.Publish(msg);

  igndbg << "Solar Panel Plugin:: Current power output: " << currentPower << " watts" << std::endl;
  igndbg << "Solar Panel Plugin:: In line of sight: " << (isInLOS ? "Yes" : "No") << std::endl;
}

//////////////////////////////////////////////////
void SolarPanelPlugin::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->dataPtr->scene != _scene)
  {
    this->dataPtr->scene = _scene;
  }
}

//////////////////////////////////////////////////
bool SolarPanelPluginPrivate::FindScene()
{
  auto loadedEngNames = ignition::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    ignwarn << "No rendering engine is loaded yet" << std::endl;
    return false;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    ignwarn << "More than one engine is available. "
            << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = ignition::rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
           << "]. Solar panel plugin won't work." << std::endl;
    return false;
  }

  if (engine->SceneCount() == 0)
  {
    igndbg << "No scene has been created yet" << std::endl;
    return false;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return false;
  }

  if (engine->SceneCount() > 1)
  {
    igndbg << "More than one scene is available. "
           << "Using scene [" << scene->Name() << "]" << std::endl;
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
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Build the prefix for the scoped name
  std::string scopedPrefix = this->modelName + "::" + this->linkName + "::";

  // Find all visual entities that are children of this link
  std::vector<std::string> scopedVisualChildren;
  _ecm.Each<ignition::gazebo::components::Visual, ignition::gazebo::components::Name, ignition::gazebo::components::ParentEntity>(
      [&](const ignition::gazebo::Entity &_entity,
          const ignition::gazebo::components::Visual *,
          const ignition::gazebo::components::Name *_name,
          const ignition::gazebo::components::ParentEntity *_parent) -> bool
      {
        if (_parent->Data() == linkEntity)
        {
          std::string scopedName = scopedPrefix + _name->Data();
          scopedVisualChildren.push_back(scopedName);
        }
        return true;
      });

  return scopedVisualChildren;
}

ignition::common::EventT<void(const ignition::rendering::ScenePtr &)>
    SolarPanelPluginPrivate::sceneEvent;

IGNITION_ADD_PLUGIN(SolarPanelPlugin, ignition::gazebo::System,
                    SolarPanelPlugin::ISystemConfigure,
                    SolarPanelPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SolarPanelPlugin, "simulation::SolarPanelPlugin")
