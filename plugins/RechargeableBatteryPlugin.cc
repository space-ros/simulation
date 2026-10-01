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

#include "RechargeableBatteryPlugin.hh"
#include "gz_compat.hh"

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/BatteryPowerLoad.hh>
#include "gz/sim/components/BatterySoC.hh"
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/common/Battery.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/battery_state.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/time.pb.h>
#include "gz/sim/Model.hh"
#include <gz/transport/Node.hh>

using namespace simulation;

/// \brief Power Source Information
struct powerSourceInfo
{
    /// \brief Power source id
    int id{0};

    /// \brief Power source power
    double nominalPower{0.0};

    /// \brief Flag to check if the power source has been updated
    bool dataUpdated{false};

    /// \brief Mutex to protect the power source state
    std::unique_ptr<std::mutex> mutex_ptr = std::make_unique<std::mutex>();
};

class simulation::RechargeableBatteryPluginPrivate
{

    /// \brief Reset the plugin
public:
    void Reset();
    /// \brief Name that identifies a battery.

    /// \brief Get the current state of charge of the battery.
    /// \return State of charge of the battery in range [0.0, 1.0].
public:
    double StateOfCharge() const;

    /// \brief Callback connected to additional topics that can start battery
    /// draining.
    /// \param[in] _data Message data.
    /// \param[in] _size Message data size.
    /// \param[in] _info Information about the message.
public:
    void OnBatteryDrainingMsg(
        const char *_data, const size_t _size,
        const gz::transport::MessageInfo &_info);

    /// \brief Callback connected to additional topics that can stop battery
    /// draining.
    /// \param[in] _data Message data.
    /// \param[in] _size Message data size.
    /// \param[in] _info Information about the message.
public:
    void OnBatteryStopDrainingMsg(
        const char *_data, const size_t _size,
        const gz::transport::MessageInfo &_info);

    /// \brief Callback connected to power source topics.
    /// \param[in] _id The id of the power source.
    /// \param[in] _msg The message containing the power source power.
public:
    void OnPowerSourceMsg(int _id,
                          const gz::msgs::Float &_msg);

    /// \brief Ignition communication node
public:
    gz::transport::Node node;

    /// \brief Battery state of charge message publisher
public:
    gz::transport::Node::Publisher statePub;

    /// \brief Total power supply publisher
public:
    gz::transport::Node::Publisher totalPowerSupplyPub;

    /// \brief Total power consumption publisher
public:
    gz::transport::Node::Publisher totalPowerConsumptionPub;

    /// \brief Battery consumer identifier.
    /// Current implementation limits one consumer (Model) per battery.
public:
    int32_t consumerId;

    /// \brief Name that identifies a battery.
public:
    std::string batteryName;

    /// \brief Pointer to battery contained in link.
public:
    gz::common::BatteryPtr battery;

    /// \brief Whether warning that battery has drained has been printed once.
public:
    bool drainPrinted{false};

    /// \brief Battery entity
public:
    gz::sim::Entity batteryEntity{gz::sim::kNullEntity};

    /// \brief modelName
public:
    std::string modelName;

    /// \brief Model entity
public:
    gz::sim::Model model{gz::sim::kNullEntity};

    /// \brief Open-circuit voltage.
    /// E(t) = e0 + e1 * Q(t) / c
public:
    double e0{0.0};

public:
    double e1{0.0};

    /// \brief Initial battery charge in Ah.
public:
    double q0{0.0};

    /// \brief Battery capacity in Ah.
public:
    double c{0.0};

    /// \brief Battery inner resistance in Ohm.
public:
    double r{0.0};

    /// \brief Current low-pass filter characteristic time in seconds [0, 1].
public:
    double tau{1.0};

    /// \brief Raw battery current in A.
public:
    double iraw{0.0};

    /// \brief Smoothed battery current in A.
public:
    double ismooth{0.0};

    /// \brief Instantaneous battery charge in Ah.
public:
    double q{0.0};

    /// \brief State of charge [0, 1].
public:
    double soc{1.0};

    /// \brief Simulation time handled during a single update.
public:
    std::chrono::steady_clock::duration stepSize;

    /// \brief Flag on whether the battery should start draining
public:
    bool startDraining = false;

    /// \brief The start time when battery starts draining in seconds
public:
    int drainStartTime = -1;

    /// \brief Book keep the last time printed, so as to not pollute dbg messages
    /// in minutes
public:
    int lastPrintTime = -1;

    /// \brief Initial power load set trough config
public:
    double initialPowerLoad = 0.0;

    /// \brief Total power supply
public:
    double totalPowerSupply = 0.0;

    /// \brief Flag to check if the battery is charging
public:
    bool isCharging = false;

    /// \brief Vector of power sources information
public:
    std::vector<powerSourceInfo> powerSourcesInfo;
};

/////////////////////////////////////////////////
RechargeableBatteryPlugin::RechargeableBatteryPlugin()
    : System(), dataPtr(std::make_unique<RechargeableBatteryPluginPrivate>())
{
}

/////////////////////////////////////////////////
RechargeableBatteryPlugin::~RechargeableBatteryPlugin()
{
    this->dataPtr->Reset();

    if (this->dataPtr->battery)
    {
        // consumer-specific
        if (this->dataPtr->consumerId != -1)
        {
            this->dataPtr->battery->RemoveConsumer(this->dataPtr->consumerId);
        }

        // This is needed so that common::Battery stops calling the update function
        //   of this object, when this object is destroyed. Else seg fault in test,
        //   though no seg fault in actual run.
        this->dataPtr->battery->ResetUpdateFunc();
    }
}

/////////////////////////////////////////////////
void RechargeableBatteryPlugin::Configure(const gz::sim::Entity &_entity,
                                          const std::shared_ptr<const sdf::Element> &_sdf,
                                          gz::sim::EntityComponentManager &_ecm,
                                          gz::sim::EventManager &_eventMgr)
{
    // Store the pointer to the model this battery is under
    auto model = gz::sim::Model(_entity);
    if (!model.Valid(_ecm))
    {
        gzerr << "Battery plugin should be attached to a model entity. "
               << "Failed to initialize." << std::endl;
        return;
    }
    this->dataPtr->model = model;
    this->dataPtr->modelName = model.Name(_ecm);

    if (_sdf->HasElement("open_circuit_voltage_constant_coef"))
        this->dataPtr->e0 = _sdf->Get<double>("open_circuit_voltage_constant_coef");

    if (_sdf->HasElement("open_circuit_voltage_linear_coef"))
        this->dataPtr->e1 = _sdf->Get<double>("open_circuit_voltage_linear_coef");

    if (_sdf->HasElement("capacity"))
        this->dataPtr->c = _sdf->Get<double>("capacity");

    if (this->dataPtr->c <= 0)
    {
        gzerr << "No <capacity> or incorrect value specified. Capacity should be "
               << "greater than 0.\n";
        return;
    }

    this->dataPtr->q0 = this->dataPtr->c;
    if (_sdf->HasElement("initial_charge"))
    {
        this->dataPtr->q0 = _sdf->Get<double>("initial_charge");
        if (this->dataPtr->q0 > this->dataPtr->c || this->dataPtr->q0 < 0)
        {
            gzerr << "<initial_charge> value should be between [0, <capacity>]."
                   << std::endl;
            this->dataPtr->q0 =
                std::max(0.0, std::min(this->dataPtr->q0, this->dataPtr->c));
            gzerr << "Setting <initial_charge> to [" << this->dataPtr->q0
                   << "] instead." << std::endl;
        }
    }

    this->dataPtr->q = this->dataPtr->q0;

    if (_sdf->HasElement("resistance"))
        this->dataPtr->r = _sdf->Get<double>("resistance");

    if (_sdf->HasElement("smooth_current_tau"))
    {
        this->dataPtr->tau = _sdf->Get<double>("smooth_current_tau");
        if (this->dataPtr->tau <= 0)
        {
            gzerr << "<smooth_current_tau> value should be positive. "
                   << "Using [1] instead." << std::endl;
            this->dataPtr->tau = 1;
        }
    }

    if (_sdf->HasElement("battery_name") && _sdf->HasElement("voltage"))
    {
        this->dataPtr->batteryName = _sdf->Get<std::string>("battery_name");
        auto initVoltage = _sdf->Get<double>("voltage");
        gzdbg << "Battery name: " << this->dataPtr->batteryName << std::endl;
        gzdbg << "Initial voltage: " << initVoltage << std::endl;

        // Create battery entity and some components
        this->dataPtr->batteryEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(this->dataPtr->batteryEntity, gz::sim::components::Name(
                                                               this->dataPtr->batteryName));
        _ecm.SetParentEntity(this->dataPtr->batteryEntity, _entity);

        // Create actual battery and assign update function
        this->dataPtr->battery = std::make_shared<gz::common::Battery>(
            this->dataPtr->batteryName, initVoltage);
        this->dataPtr->battery->Init();
        // print battery voltage
        gzdbg << "Battery voltage: " << this->dataPtr->battery->Voltage() << std::endl;
        this->dataPtr->battery->SetUpdateFunc(
            std::bind(&RechargeableBatteryPlugin::OnUpdateVoltage, this,
                      std::placeholders::_1));
    }
    else
    {
        gzerr << "No <battery_name> or <voltage> specified. Both are required.\n";
        return;
    }

    // Consumer-specific
    if (_sdf->HasElement("power_load"))
    {
        this->dataPtr->initialPowerLoad = _sdf->Get<double>("power_load");
        this->dataPtr->consumerId = this->dataPtr->battery->AddConsumer();
        bool success = this->dataPtr->battery->SetPowerLoad(
            this->dataPtr->consumerId, this->dataPtr->initialPowerLoad);
        if (!success)
            gzerr << "Failed to set consumer power load." << std::endl;
    }
    else
    {
        gzwarn << "Required attribute power_load missing "
                << "in BatteryPlugin SDF" << std::endl;
    }
    if (_sdf->HasElement("start_draining"))
        this->dataPtr->startDraining = _sdf->Get<bool>("start_draining");

    // Subscribe to power draining topics, if any.
    if (_sdf->HasElement("power_draining_topic"))
    {
        sdf::ElementConstPtr sdfElem = _sdf->FindElement("power_draining_topic");
        while (sdfElem)
        {
            const auto &topic = sdfElem->Get<std::string>();
            this->dataPtr->node.SubscribeRaw(topic,
                                             std::bind(&RechargeableBatteryPluginPrivate::OnBatteryDrainingMsg,
                                                       this->dataPtr.get(), std::placeholders::_1, std::placeholders::_2,
                                                       std::placeholders::_3));
            gzmsg << "RechargeableBatteryPlugin subscribes to power draining topic ["
                   << topic << "]." << std::endl;
            sdfElem = sdfElem->GetNextElement("power_draining_topic");
        }
    }

    // Subscribe to stop power draining topics, if any.
    if (_sdf->HasElement("stop_power_draining_topic"))
    {
        sdf::ElementConstPtr sdfElem =
            _sdf->FindElement("stop_power_draining_topic");
        while (sdfElem)
        {
            const auto &topic = sdfElem->Get<std::string>();
            this->dataPtr->node.SubscribeRaw(topic,
                                             std::bind(&RechargeableBatteryPluginPrivate::OnBatteryStopDrainingMsg,
                                                       this->dataPtr.get(), std::placeholders::_1, std::placeholders::_2,
                                                       std::placeholders::_3));
            gzmsg << "RechargeableBatteryPlugin subscribes to stop power draining topic ["
                   << topic << "]." << std::endl;
            sdfElem = sdfElem->GetNextElement("power_draining_topic");
        }
    }

    // subscriber to all power sources topics
    if (_sdf->HasElement("power_source"))
    {
        sdf::ElementConstPtr powerSourceElem = _sdf->FindElement("power_source");
        int id = 0;
        while (powerSourceElem)
        {
            const auto &topic = powerSourceElem->Get<std::string>();
            std::string stateTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/" +
                                   topic};
            auto validPowerSourceTopic = gz::transport::TopicUtils::AsValidTopic(stateTopic);
            if (validPowerSourceTopic.empty())
            {
                gzerr << "Failed to create valid topic. Not valid: ["
                       << topic << "]" << std::endl;
                return;
            }
            powerSourceInfo powerSource;
            powerSource.id = id;
            powerSource.nominalPower = 0.0;
            powerSource.dataUpdated = false;
            std::function<void(const gz::msgs::Float &)> callback = std::bind(&RechargeableBatteryPluginPrivate::OnPowerSourceMsg,
                                                                                    this->dataPtr.get(), id, std::placeholders::_1);
            this->dataPtr->node.Subscribe(validPowerSourceTopic, callback);
            gzmsg << "RechargeableBatteryPlugin subscribes to power source topic ["
                   << validPowerSourceTopic << "]." << std::endl;
            this->dataPtr->powerSourcesInfo.emplace_back(std::move(powerSource));
            powerSourceElem = powerSourceElem->GetNextElement("power_source");
            id++;
        }
    }
    else
    {
        gzerr << "No power source topic specified." << std::endl;
    }

    gzmsg << "RechargeableBatteryPlugin configured. Battery name: "
           << this->dataPtr->battery->Name() << std::endl;
    gzdbg << "Battery initial voltage: " << this->dataPtr->battery->InitVoltage()
           << std::endl;

    this->dataPtr->soc = this->dataPtr->q / this->dataPtr->c;
    // Initialize battery with initial calculated state of charge
    _ecm.CreateComponent(this->dataPtr->batteryEntity,
                         gz::sim::components::BatterySoC(this->dataPtr->soc));

    // Setup battery state topic
    std::string stateTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
                           "/battery/" + this->dataPtr->battery->Name() + "/state"};

    auto validStateTopic = gz::transport::TopicUtils::AsValidTopic(stateTopic);
    if (validStateTopic.empty())
    {
        gzerr << "Failed to create valid state topic ["
               << stateTopic << "]" << std::endl;
        return;
    }

    gz::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(50);
    this->dataPtr->statePub = this->dataPtr->node.Advertise<gz::msgs::BatteryState>(
        validStateTopic, opts);

    // Setup total power supply topic
    std::string totalPowerSupplyTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
                                      "/battery/" + this->dataPtr->battery->Name() + "/total_power_supply"};

    auto validTotalPowerSupplyTopic = gz::transport::TopicUtils::AsValidTopic(totalPowerSupplyTopic);
    if (validTotalPowerSupplyTopic.empty())
    {
        gzerr << "Failed to create valid total power supply topic ["
               << totalPowerSupplyTopic << "]" << std::endl;
        return;
    }

    this->dataPtr->totalPowerSupplyPub = this->dataPtr->node.Advertise<gz::msgs::Float>(
        validTotalPowerSupplyTopic);

    // Setup total power consumption topic
    std::string totalPowerConsumptionTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
                                           "/battery/" + this->dataPtr->battery->Name() + "/total_power_consumption"};
    
    auto validTotalPowerConsumptionTopic = gz::transport::TopicUtils::AsValidTopic(totalPowerConsumptionTopic);

    if (validTotalPowerConsumptionTopic.empty())
    {
        gzerr << "Failed to create valid total power consumption topic ["
               << totalPowerConsumptionTopic << "]" << std::endl;
        return;
    }

    this->dataPtr->totalPowerConsumptionPub = this->dataPtr->node.Advertise<gz::msgs::Float>(
        validTotalPowerConsumptionTopic);
}

/////////////////////////////////////////////////
void RechargeableBatteryPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    GZ_PROFILE("RechargeableBatteryPlugin::PreUpdate");
    // Recalculate total power load among all consumers
    double totalPowerLoad = this->dataPtr->initialPowerLoad;
    _ecm.Each<gz::sim::components::BatteryPowerLoad>(
        [&](const gz::sim::Entity & /*_entity*/,
            const gz::sim::components::BatteryPowerLoad *_batteryPowerLoadInfo) -> bool
        {
            if (_batteryPowerLoadInfo->Data().batteryId ==
                this->dataPtr->batteryEntity)
            {
                totalPowerLoad = totalPowerLoad +
                                 _batteryPowerLoadInfo->Data().batteryPowerLoad;
            }
            return true;
        });

    // Publish total power consumption
    gz::msgs::Float totalPowerConsumptionMsg;
    totalPowerConsumptionMsg.set_data(totalPowerLoad);
    this->dataPtr->totalPowerConsumptionPub.Publish(totalPowerConsumptionMsg);

    bool success = this->dataPtr->battery->SetPowerLoad(
        this->dataPtr->consumerId, totalPowerLoad);
    if (!success)
        gzerr << "Failed to set consumer power load." << std::endl;

    // start draining the battery if the robot has started moving
    if (!this->dataPtr->startDraining)
    {
        const std::vector<gz::sim::Entity> &joints =
            _ecm.ChildrenByComponents(this->dataPtr->model.Entity(), gz::sim::components::Joint());

        for (gz::sim::Entity jointEntity : joints)

        {
            const auto *jointVelocityCmd =
                _ecm.Component<gz::sim::components::JointVelocityCmd>(jointEntity);
            if (jointVelocityCmd)
            {
                for (double jointVel : jointVelocityCmd->Data())
                {
                    if (fabs(static_cast<float>(jointVel)) > 0.01)
                    {
                        this->dataPtr->startDraining = true;
                        break;
                    }
                }
            }

            const auto *jointForceCmd =
                _ecm.Component<gz::sim::components::JointForceCmd>(jointEntity);
            if (jointForceCmd)
            {
                for (double jointForce : jointForceCmd->Data())
                {
                    if (fabs(static_cast<float>(jointForce)) > 0.01)
                    {
                        this->dataPtr->startDraining = true;
                        break;
                    }
                }
            }
        }
    }
}

///////////////////////////////////////////////
void RechargeableBatteryPlugin::Update(const gz::sim::UpdateInfo &_info,
                                       gz::sim::EntityComponentManager &_ecm)
{
    GZ_PROFILE("RechargeableBatteryPlugin::Update");
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        gzwarn << "Detected jump back in time ["
                << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
                << "s]. System may not work properly." << std::endl;
    }

    if (_info.paused)
        return;

    if (!this->dataPtr->startDraining)
        return;

    // Find the time at which battery starts to drain
    int simTime = static_cast<int>(
        std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
    if (this->dataPtr->drainStartTime == -1)
        this->dataPtr->drainStartTime = simTime;

    // Print drain time in minutes
    int drainTime = (simTime - this->dataPtr->drainStartTime) / 60;
    if (drainTime != this->dataPtr->lastPrintTime)
    {
        this->dataPtr->lastPrintTime = drainTime;
        gzdbg << "[Battery Plugin] Battery drain: " << drainTime << " minutes passed.\n";
    }

    // update step size
    this->dataPtr->stepSize = _info.dt;

    // Sanity check: tau should be in the range [dt, +inf)
    double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>(
                     this->dataPtr->stepSize)
                     .count()) *
                1e-9;
    if (this->dataPtr->tau < dt)
    {
        gzerr << "<smooth_current_tau> should be in the range [dt, +inf) but is "
               << "configured with [" << this->dataPtr->tau << "]. We'll be using "
               << "[" << dt << "] instead" << std::endl;
        this->dataPtr->tau = dt;
    }

    if (this->dataPtr->battery)
    {
        // Update battery component
        this->dataPtr->battery->Update();
        auto *batteryComp = _ecm.Component<gz::sim::components::BatterySoC>(
            this->dataPtr->batteryEntity);

        batteryComp->Data() = this->dataPtr->StateOfCharge();
    }
}

/////////////////////////////////////////////////
void RechargeableBatteryPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                           const gz::sim::EntityComponentManager &_ecm)
{
    GZ_PROFILE("RechargeableBatteryPlugin::PostUpdate");
    // Nothing left to do if paused or the publisher wasn't created.
    if (_info.paused || !this->dataPtr->statePub)
        return;

    // Publish battery state
    gz::msgs::BatteryState msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(gz::sim::convert<gz::msgs::Time>(_info.simTime));
    msg.set_voltage(this->dataPtr->battery->Voltage());
    msg.set_current(this->dataPtr->ismooth);
    msg.set_charge(this->dataPtr->q);
    msg.set_capacity(this->dataPtr->c);
    msg.set_percentage(this->dataPtr->soc * 100);

    if (this->dataPtr->isCharging)
        msg.set_power_supply_status(gz::msgs::BatteryState::CHARGING);
    else if (this->dataPtr->startDraining)
        msg.set_power_supply_status(gz::msgs::BatteryState::DISCHARGING);
    else if (!this->dataPtr->StateOfCharge() > 0.9)
        msg.set_power_supply_status(gz::msgs::BatteryState::FULL);
    else
        msg.set_power_supply_status(gz::msgs::BatteryState::NOT_CHARGING);

    this->dataPtr->statePub.Publish(msg);
}

/////////////////////////////////////////////////
double RechargeableBatteryPlugin::OnUpdateVoltage(const gz::common::Battery *_battery)
{
    GZ_ASSERT(_battery != nullptr, "Battery pointer is null");

    if (fabs(_battery->Voltage()) < 1e-3)
        return 0.0;
    if (this->dataPtr->StateOfCharge() <= 0)
        return _battery->Voltage();

    auto prevSocInt = static_cast<int>(this->dataPtr->StateOfCharge() * 100);

    // seconds
    double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>(
                     this->dataPtr->stepSize)
                     .count()) *
                1e-9;
    double totalpower = 0.0;
    double k = dt / this->dataPtr->tau;

    if (this->dataPtr->startDraining)
    {
        for (auto powerLoad : _battery->PowerLoads())
            totalpower += powerLoad.second;
    }

    this->dataPtr->iraw = totalpower / _battery->Voltage();

    // Update total power supply
    for (auto &powerSource : this->dataPtr->powerSourcesInfo)
    {
        std::lock_guard<std::mutex> lock(*(powerSource.mutex_ptr));
        if (powerSource.dataUpdated)
        {
            powerSource.dataUpdated = false;
            this->dataPtr->totalPowerSupply += powerSource.nominalPower;
        }
    }

    // Publish total power supply
    gz::msgs::Float totalPowerSupplyMsg;
    totalPowerSupplyMsg.set_data(this->dataPtr->totalPowerSupply);
    this->dataPtr->totalPowerSupplyPub.Publish(totalPowerSupplyMsg);

    // check if the total powersupply is not zero
    this->dataPtr->isCharging = this->dataPtr->totalPowerSupply > 1e-9 && this->dataPtr->StateOfCharge() < 0.9;

    // compute current due to power sources
    float powerSourceCurrent = 0.0;
    if (this->dataPtr->isCharging)
    {
        powerSourceCurrent = this->dataPtr->totalPowerSupply / _battery->Voltage();
        this->dataPtr->iraw -= powerSourceCurrent;
    }
    // reset total power supply to zero
    this->dataPtr->totalPowerSupply = 0.0;

    this->dataPtr->ismooth = this->dataPtr->ismooth + k *
                                                          (this->dataPtr->iraw - this->dataPtr->ismooth);

    // Convert dt to hours
    this->dataPtr->q = this->dataPtr->q - ((dt * this->dataPtr->ismooth) /
                                           3600.0);

    // open circuit voltage
    double voltage = this->dataPtr->e0 + this->dataPtr->e1 * (1 - this->dataPtr->q / this->dataPtr->c) - this->dataPtr->r * this->dataPtr->ismooth;

    // Estimate state of charge
    this->dataPtr->soc = this->dataPtr->q / this->dataPtr->c;

    // Throttle debug messages
    auto socInt = static_cast<int>(this->dataPtr->StateOfCharge() * 100);

    if (socInt % 10 == 0 && socInt != prevSocInt)
    {
        gzdbg << "Battery: " << this->dataPtr->battery->Name() << std::endl;
        gzdbg << "PowerLoads().size(): " << _battery->PowerLoads().size()
               << std::endl;
        gzdbg << "charging current: " << powerSourceCurrent << std::endl;
        gzdbg << "voltage: " << voltage << std::endl;
        gzdbg << "state of charge: " << this->dataPtr->StateOfCharge()
               << " (q " << this->dataPtr->q << ")" << std::endl
               << std::endl;
    }
    if (this->dataPtr->StateOfCharge() < 0 && !this->dataPtr->drainPrinted)
    {
        gzwarn << "Model " << this->dataPtr->modelName << " out of battery.\n";
        this->dataPtr->drainPrinted = true;
    }

    return voltage;
}

/////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::Reset()
{
    this->totalPowerSupply = 0.0;
    this->ismooth = 0.0;
    this->iraw = 0.0;
    this->q = this->q0;
    this->startDraining = false;
}

//////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::OnBatteryDrainingMsg(
    const char *, const size_t, const gz::transport::MessageInfo &)
{
    this->startDraining = true;
}

//////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::OnBatteryStopDrainingMsg(
    const char *, const size_t, const gz::transport::MessageInfo &)
{
    this->startDraining = false;
}

//////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::OnPowerSourceMsg(int _id,
                                                        const gz::msgs::Float &_msg)
{
    std::lock_guard<std::mutex> lock(*(this->powerSourcesInfo[_id].mutex_ptr));
    this->powerSourcesInfo[_id].nominalPower = _msg.data();
    this->powerSourcesInfo[_id].dataUpdated = true;
}

//////////////////////////////////////////////////
double RechargeableBatteryPluginPrivate::StateOfCharge() const
{
    return this->soc;
}

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(RechargeableBatteryPlugin,
                    gz::sim::System,
                    RechargeableBatteryPlugin::ISystemConfigure,
                    RechargeableBatteryPlugin::ISystemPreUpdate,
                    RechargeableBatteryPlugin::ISystemUpdate,
                    RechargeableBatteryPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(RechargeableBatteryPlugin, "simulation::RechargeableBatteryPlugin")
