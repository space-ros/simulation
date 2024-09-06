#ifndef SOLAR_PANEL_PLUGIN_HH_
#define SOLAR_PANEL_PLUGIN_HH_

#include <string>
#include <mutex>
#include <memory>

#include <ignition/gazebo/System.hh>
#include <ignition/rendering/Scene.hh>

namespace simulation
{
  // Forward declaration
  class SolarPanelPluginPrivate;

  /// \brief SolarPanelPlugin SolarPanelPlugin.hh
  /// \brief A plugin that simulates a solar panel
  class SolarPanelPlugin : public ignition::gazebo::System,
                           public ignition::gazebo::ISystemConfigure,
                           public ignition::gazebo::ISystemPostUpdate
  {

    /// \brief Constructor
  public:
    SolarPanelPlugin();

    /// \brief Destructor
  public:
    ~SolarPanelPlugin() override;

    /// Documentation inherited
  public:
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    /// Documentation inherited
  public:
    void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                    const ignition::gazebo::EntityComponentManager &_ecm) final;

    /// \brief Set the scene
  public:
    void SetScene(ignition::rendering::ScenePtr _scene);

    /// \brief Private data pointer
  private:
    std::unique_ptr<SolarPanelPluginPrivate> dataPtr;
  };
}
#endif // SOLAR_PANEL_PLUGIN_HH_
