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

#ifndef SOLAR_PANEL_PLUGIN_HH_
#define SOLAR_PANEL_PLUGIN_HH_

#include <string>
#include <mutex>
#include <memory>

#include <gz/sim/System.hh>
#include <gz/rendering/Scene.hh>

namespace simulation
{
  // Forward declaration
  class SolarPanelPluginPrivate;

  /// \brief SolarPanelPlugin SolarPanelPlugin.hh
  /// \brief A plugin that simulates a solar panel
  class SolarPanelPlugin : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPostUpdate
  {

    /// \brief Constructor
  public:
    SolarPanelPlugin();

    /// \brief Destructor
  public:
    ~SolarPanelPlugin() override;

    /// Documentation inherited
  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    /// Documentation inherited
  public:
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) final;

    /// \brief Set the scene
  public:
    void SetScene(gz::rendering::ScenePtr _scene);

    /// \brief Private data pointer
  private:
    std::unique_ptr<SolarPanelPluginPrivate> dataPtr;
  };
}
#endif // SOLAR_PANEL_PLUGIN_HH_
