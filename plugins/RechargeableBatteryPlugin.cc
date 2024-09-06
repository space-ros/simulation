#include "RechargeableBatteryPlugin.hh"

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/BatteryPowerLoad.hh>
#include "ignition/gazebo/components/BatterySoC.hh"
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <gz/common/Battery.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <ignition/msgs/battery_state.pb.h>
#include "gz/sim/Model.hh"
#include <ignition/transport/Node.hh>

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
        const ignition::transport::MessageInfo &_info);

    /// \brief Callback connected to additional topics that can stop battery
    /// draining.
    /// \param[in] _data Message data.
    /// \param[in] _size Message data size.
    /// \param[in] _info Information about the message.
public:
    void OnBatteryStopDrainingMsg(
        const char *_data, const size_t _size,
        const ignition::transport::MessageInfo &_info);

    /// \brief Callback connected to power source topics.
    /// \param[in] _id The id of the power source.
    /// \param[in] _msg The message containing the power source power.
public:
    void OnPowerSourceMsg(int _id,
                          const ignition::msgs::Float &_msg);

    /// \brief Ignition communication node
public:
    ignition::transport::Node node;

    /// \brief Battery state of charge message publisher
public:
    ignition::transport::Node::Publisher statePub;

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
    ignition::gazebo::Entity batteryEntity{ignition::gazebo::kNullEntity};

    /// \brief modelName
public:
    std::string modelName;

    /// \brief Model entity
public:
    gz::sim::Model model{ignition::gazebo::kNullEntity};

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
void RechargeableBatteryPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                          const std::shared_ptr<const sdf::Element> &_sdf,
                                          ignition::gazebo::EntityComponentManager &_ecm,
                                          ignition::gazebo::EventManager &_eventMgr)
{
    // Store the pointer to the model this battery is under
    auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm))
    {
        ignerr << "Battery plugin should be attached to a model entity. "
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
        ignerr << "No <capacity> or incorrect value specified. Capacity should be "
               << "greater than 0.\n";
        return;
    }

    this->dataPtr->q0 = this->dataPtr->c;
    if (_sdf->HasElement("initial_charge"))
    {
        this->dataPtr->q0 = _sdf->Get<double>("initial_charge");
        if (this->dataPtr->q0 > this->dataPtr->c || this->dataPtr->q0 < 0)
        {
            ignerr << "<initial_charge> value should be between [0, <capacity>]."
                   << std::endl;
            this->dataPtr->q0 =
                std::max(0.0, std::min(this->dataPtr->q0, this->dataPtr->c));
            ignerr << "Setting <initial_charge> to [" << this->dataPtr->q0
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
            ignerr << "<smooth_current_tau> value should be positive. "
                   << "Using [1] instead." << std::endl;
            this->dataPtr->tau = 1;
        }
    }

    if (_sdf->HasElement("battery_name") && _sdf->HasElement("voltage"))
    {
        this->dataPtr->batteryName = _sdf->Get<std::string>("battery_name");
        auto initVoltage = _sdf->Get<double>("voltage");
        igndbg << "Battery name: " << this->dataPtr->batteryName << std::endl;
        igndbg << "Initial voltage: " << initVoltage << std::endl;

        // Create battery entity and some components
        this->dataPtr->batteryEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(this->dataPtr->batteryEntity, ignition::gazebo::components::Name(
                                                               this->dataPtr->batteryName));
        _ecm.SetParentEntity(this->dataPtr->batteryEntity, _entity);

        // Create actual battery and assign update function
        this->dataPtr->battery = std::make_shared<ignition::common::Battery>(
            this->dataPtr->batteryName, initVoltage);
        this->dataPtr->battery->Init();
        // print battery voltage
        igndbg << "Battery voltage: " << this->dataPtr->battery->Voltage() << std::endl;
        this->dataPtr->battery->SetUpdateFunc(
            std::bind(&RechargeableBatteryPlugin::OnUpdateVoltage, this,
                      std::placeholders::_1));
    }
    else
    {
        ignerr << "No <battery_name> or <voltage> specified. Both are required.\n";
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
            ignerr << "Failed to set consumer power load." << std::endl;
    }
    else
    {
        ignwarn << "Required attribute power_load missing "
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
            ignmsg << "RechargeableBatteryPlugin subscribes to power draining topic ["
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
            ignmsg << "RechargeableBatteryPlugin subscribes to stop power draining topic ["
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
            auto validPowerSourceTopic = ignition::transport::TopicUtils::AsValidTopic(stateTopic);
            if (validPowerSourceTopic.empty())
            {
                ignerr << "Failed to create valid topic. Not valid: ["
                       << topic << "]" << std::endl;
                return;
            }
            powerSourceInfo powerSource;
            powerSource.id = id;
            powerSource.nominalPower = 0.0;
            powerSource.dataUpdated = false;
            std::function<void(const ignition::msgs::Float &)> callback = std::bind(&RechargeableBatteryPluginPrivate::OnPowerSourceMsg,
                                                                                    this->dataPtr.get(), id, std::placeholders::_1);
            this->dataPtr->node.Subscribe(validPowerSourceTopic, callback);
            ignmsg << "RechargeableBatteryPlugin subscribes to power source topic ["
                   << validPowerSourceTopic << "]." << std::endl;
            this->dataPtr->powerSourcesInfo.emplace_back(std::move(powerSource));
            powerSourceElem = powerSourceElem->GetNextElement("power_source");
            id++;
        }
    }
    else
    {
        ignerr << "No power source topic specified." << std::endl;
    }

    ignmsg << "RechargeableBatteryPlugin configured. Battery name: "
           << this->dataPtr->battery->Name() << std::endl;
    igndbg << "Battery initial voltage: " << this->dataPtr->battery->InitVoltage()
           << std::endl;

    this->dataPtr->soc = this->dataPtr->q / this->dataPtr->c;
    // Initialize battery with initial calculated state of charge
    _ecm.CreateComponent(this->dataPtr->batteryEntity,
                         ignition::gazebo::components::BatterySoC(this->dataPtr->soc));

    // Setup battery state topic
    std::string stateTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
                           "/battery/" + this->dataPtr->battery->Name() + "/state"};

    auto validStateTopic = ignition::transport::TopicUtils::AsValidTopic(stateTopic);
    if (validStateTopic.empty())
    {
        ignerr << "Failed to create valid state topic ["
               << stateTopic << "]" << std::endl;
        return;
    }

    ignition::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(50);
    this->dataPtr->statePub = this->dataPtr->node.Advertise<ignition::msgs::BatteryState>(
        validStateTopic, opts);
}

/////////////////////////////////////////////////
void RechargeableBatteryPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("RechargeableBatteryPlugin::PreUpdate");
    // Recalculate total power load among all consumers
    double totalPowerLoad = this->dataPtr->initialPowerLoad;
    _ecm.Each<ignition::gazebo::components::BatteryPowerLoad>(
        [&](const ignition::gazebo::Entity & /*_entity*/,
            const ignition::gazebo::components::BatteryPowerLoad *_batteryPowerLoadInfo) -> bool
        {
            if (_batteryPowerLoadInfo->Data().batteryId ==
                this->dataPtr->batteryEntity)
            {
                totalPowerLoad = totalPowerLoad +
                                 _batteryPowerLoadInfo->Data().batteryPowerLoad;
            }
            return true;
        });

    bool success = this->dataPtr->battery->SetPowerLoad(
        this->dataPtr->consumerId, totalPowerLoad);
    if (!success)
        ignerr << "Failed to set consumer power load." << std::endl;

    // start draining the battery if the robot has started moving
    if (!this->dataPtr->startDraining)
    {
        const std::vector<ignition::gazebo::Entity> &joints =
            _ecm.ChildrenByComponents(this->dataPtr->model.Entity(), ignition::gazebo::components::Joint());

        for (ignition::gazebo::Entity jointEntity : joints)

        {
            const auto *jointVelocityCmd =
                _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
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
                _ecm.Component<ignition::gazebo::components::JointForceCmd>(jointEntity);
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
void RechargeableBatteryPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
                                       ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("RechargeableBatteryPlugin::Update");
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        ignwarn << "Detected jump back in time ["
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
        igndbg << "[Battery Plugin] Battery drain: " << drainTime << " minutes passed.\n";
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
        ignerr << "<smooth_current_tau> should be in the range [dt, +inf) but is "
               << "configured with [" << this->dataPtr->tau << "]. We'll be using "
               << "[" << dt << "] instead" << std::endl;
        this->dataPtr->tau = dt;
    }

    if (this->dataPtr->battery)
    {
        // Update battery component
        this->dataPtr->battery->Update();
        auto *batteryComp = _ecm.Component<ignition::gazebo::components::BatterySoC>(
            this->dataPtr->batteryEntity);

        batteryComp->Data() = this->dataPtr->StateOfCharge();
    }
}

/////////////////////////////////////////////////
void RechargeableBatteryPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                           const ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("RechargeableBatteryPlugin::PostUpdate");
    // Nothing left to do if paused or the publisher wasn't created.
    if (_info.paused || !this->dataPtr->statePub)
        return;

    // Publish battery state
    ignition::msgs::BatteryState msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(ignition::gazebo::convert<ignition::msgs::Time>(_info.simTime));
    msg.set_voltage(this->dataPtr->battery->Voltage());
    msg.set_current(this->dataPtr->ismooth);
    msg.set_charge(this->dataPtr->q);
    msg.set_capacity(this->dataPtr->c);
    msg.set_percentage(this->dataPtr->soc * 100);

    if (this->dataPtr->isCharging)
        msg.set_power_supply_status(ignition::msgs::BatteryState::CHARGING);
    else if (this->dataPtr->startDraining)
        msg.set_power_supply_status(ignition::msgs::BatteryState::DISCHARGING);
    else if (!this->dataPtr->StateOfCharge() > 0.9)
        msg.set_power_supply_status(ignition::msgs::BatteryState::FULL);
    else
        msg.set_power_supply_status(ignition::msgs::BatteryState::NOT_CHARGING);

    this->dataPtr->statePub.Publish(msg);
}

/////////////////////////////////////////////////
double RechargeableBatteryPlugin::OnUpdateVoltage(const ignition::common::Battery *_battery)
{
    IGN_ASSERT(_battery != nullptr, "Battery pointer is null");

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
        igndbg << "Battery: " << this->dataPtr->battery->Name() << std::endl;
        igndbg << "PowerLoads().size(): " << _battery->PowerLoads().size()
               << std::endl;
        igndbg << "charging current: " << powerSourceCurrent << std::endl;
        igndbg << "voltage: " << voltage << std::endl;
        igndbg << "state of charge: " << this->dataPtr->StateOfCharge()
               << " (q " << this->dataPtr->q << ")" << std::endl
               << std::endl;
    }
    if (this->dataPtr->StateOfCharge() < 0 && !this->dataPtr->drainPrinted)
    {
        ignwarn << "Model " << this->dataPtr->modelName << " out of battery.\n";
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
    const char *, const size_t, const ignition::transport::MessageInfo &)
{
    this->startDraining = true;
}

//////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::OnBatteryStopDrainingMsg(
    const char *, const size_t, const ignition::transport::MessageInfo &)
{
    this->startDraining = false;
}

//////////////////////////////////////////////////
void RechargeableBatteryPluginPrivate::OnPowerSourceMsg(int _id,
                                                        const ignition::msgs::Float &_msg)
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

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(RechargeableBatteryPlugin,
                    ignition::gazebo::System,
                    RechargeableBatteryPlugin::ISystemConfigure,
                    RechargeableBatteryPlugin::ISystemPreUpdate,
                    RechargeableBatteryPlugin::ISystemUpdate,
                    RechargeableBatteryPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RechargeableBatteryPlugin, "simulation::RechargeableBatteryPlugin")
