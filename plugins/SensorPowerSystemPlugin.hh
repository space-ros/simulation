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
