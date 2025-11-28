/*
 * Copyright (C) 2024 Stevedan Ogochukwu Omodolor Omodia
 * Copyright (C) 2024 Robin Baran
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

#ifndef SENSOR_POWER_SYSTEM_PLUGIN_HH_
#define SENSOR_POWER_SYSTEM_PLUGIN_HH_



#include <ignition/gazebo/System.hh>


namespace simulation
{
    // Forward declaration
    class SensorPowerSystemPrivate;

    /// \brief A plugin to manage the power of sensors. It reads the power load of each sensor
    /// and the battery that powers it. It then adds it as a consumers to the battery.
    /// In addition, a topic is exposed to activate or deactivate the sensors.
    /// - `/model/{model_name}/sensor/{sensor_name}/activate`: A topic to activate or deactivate the sensor
    ///
    /// This system does not require any configuration. Although it requires the 
    /// presence of a Battery plugin in the entity and each sensors should contain the 
    /// following parameters:
    /// - `battery_name`: The name of the battery that powers the sensor
    /// - `power_load`: The power load of the sensor
    /// TODO(@stevedanomodolor): Ideally this should be part of the sensors system plugin
    /// This way not only can we manage the power load of the sensors but also desable the sensors
    /// both for the rendering and the physics engine. Lack of public API to do this is the reason
    /// why this is a separate plugin.

class SensorPowerSystemPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemPostUpdate
      {
            // Constructor
        public:
            SensorPowerSystemPlugin();

            // Destructor
        public:
            ~SensorPowerSystemPlugin() override;

            /// Documentation Inherited
        public:
            void Configure(const ignition::gazebo::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        ignition::gazebo::EntityComponentManager &_ecm,
                        ignition::gazebo::EventManager &_eventMgr) final;
            /// Documentation Inherited
        public:
            void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                        ignition::gazebo::EntityComponentManager &_ecm) override;

            /// Documentation Inherited
        public:
            void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                            const ignition::gazebo::EntityComponentManager &_ecm) override;
            /// \brief Private data pointer
        private:
            std::unique_ptr<SensorPowerSystemPrivate> dataPtr;
      };
}

#endif // SENSOR_POWER_SYSTEM_PLUGIN_HH_
