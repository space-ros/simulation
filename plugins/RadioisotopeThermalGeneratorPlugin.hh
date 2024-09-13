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

#ifndef Radioisotope_Thermal_Generator_Plugin_HH_
#define Radioisotope_Thermal_Generator_Plugin_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace simulation
{

  // Forward declaration
  class RadioisotopeThermalGeneratorPluginPrivate;

  /// \class RadioisotopeThermalGenerator RadioisotopeThermalGenerator.hh
  /// \brief Radioisotope Thermal Generator plugin. plugin allows to simulate an radioisotope thermal generator power output. It provides a constant power supply.

  /// ## System Parameters
  ///
  /// - `link_name`: The name of the link where the radioisotope thermal generator is attached.
  /// - `nominal_power`: The nominal power output of the radioisotope thermal generator.
  class RadioisotopeThermalGeneratorPlugin : public ignition::gazebo::System,
                                             public ignition::gazebo::ISystemConfigure,
                                             public ignition::gazebo::ISystemPostUpdate
  {

    /// \brief Constructor
  public:
    RadioisotopeThermalGeneratorPlugin();

    /// \brief Destructor
  public:
    ~RadioisotopeThermalGeneratorPlugin() override;

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

    /// \brief Private data pointer
  private:
    std::unique_ptr<RadioisotopeThermalGeneratorPluginPrivate> dataPtr;
  };
}
#endif // Radioisotope_Thermal_Generator_Plugin_HH_
