#ifndef RECHARGEABLE_BATTERY_PLUGIN_HH_
#define RECHARGEABLE_BATTERY_PLUGIN_HH_

#include <memory>
#include <ignition/gazebo/System.hh>
#include <ignition/common/Battery.hh>


namespace simulation
{
    // Forward declaration
    class RechargeableBatteryPluginPrivate;

    /// \brief A plugin to simulate rechargeable batteries usage.
    /// This was adapted from the LinearBatteryPlugin from Ignition Gazebo.
    ///
    /// This system processes the following sdf parameters:
    /// - `<battery_name>`: The name of the battery(required)
    /// - `<voltage>`: The initial voltage of the battery(required)
    /// - `<open_circuit_voltage_constant_coef>`: Voltage at full charge
    /// - `<open_circuit_voltage_linear_coef>`: Amount of voltage drop when no charge
    /// - `<initial_charge>`: The initial charge of the battery (Ah)
    /// - `<capacity>`: Total charge that the battery can hold (Ah)
    /// - `<resistance>`: The internal resistance of the battery (Ohm)
    /// - `<smooth_current_tau>`: Coefficient for smoothing the current [0, 1]
    /// - `<power_load>`: The power load on the battery (Watts)
    /// - `<start_draining>`: Whether to start draining the battery
    /// - `<power_draining_topic>`: This is to start draining the battery
    /// - `<stop_power_draining_topic>`: This is to stop draining the battery
    /// - `<power_source>`: This is to subscribe to power sources topics. Repeat as many times as needed with the same name

    class RechargeableBatteryPlugin
        : public ignition::gazebo::System,
          public ignition::gazebo::ISystemConfigure,
          public ignition::gazebo::ISystemPreUpdate,
          public ignition::gazebo::ISystemUpdate,
          public ignition::gazebo::ISystemPostUpdate
    {
        /// \brief Constructor
    public:
        RechargeableBatteryPlugin();

        /// \brief Destructor
    public:
        ~RechargeableBatteryPlugin() override;

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

        // Documentation Inherited
    public:
        void Update(const ignition::gazebo::UpdateInfo &_info,
                    ignition::gazebo::EntityComponentManager &_ecm) override;

        /// Documentation Inherited
    public:
        void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                        const ignition::gazebo::EntityComponentManager &_ecm) override;

        /// \brief Callback for Battery Update events.
        /// \param[in] _battery Pointer to the battery that is to be updated.
        /// \return The new voltage.
    private:
        double OnUpdateVoltage(const ignition::common::Battery *_battery);

        /// \brief Private data pointer
    private:
        std::unique_ptr<RechargeableBatteryPluginPrivate> dataPtr;
    };

}
#endif // RECHARGEABLE_BATTERY_PLUGIN_HH_
