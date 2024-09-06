#include "SensorPowerSystemPlugin.hh"
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Camera.hh>
#include <ignition/gazebo/components/Imu.hh>
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/BatteryPowerLoad.hh"
#include <sdf/Sensor.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>
#include "gz/sim/Model.hh"
#include <gz/common/Util.hh>
#include <gz/sensors/Manager.hh>
#include "ignition/sensors/Sensor.hh"

using namespace simulation;

/// \brief Sensor information
struct SensorInfo
{
    /// \brief Sensor id
    int id{0};

    /// \brief Sensor name
    std::string name{""};

    /// \brief Keep track of the sensor state
    bool enableSensor{false};

    /// \brief Power load of the sensor
    double powerLoad{0.0};

    /// \brief Battery name
    std::string batteryName{""};

    /// \brief Battery entity
    ignition::gazebo::Entity batteryEntity{ignition::gazebo::kNullEntity};

    /// \brief Battery consumer entity
    ignition::gazebo::Entity batteryConsumerEntity{ignition::gazebo::kNullEntity};

    /// \brief Flag to check if the battery exists
    bool batteryExist{false};

    /// \brief Flag to check if it has enough battery
    bool hasEnoughBattery{true};

    /// \brief Mutex to protect the sensor state
    std::unique_ptr<std::mutex> mutex_ptr = std::make_unique<std::mutex>();

    /// \brief Flag to check if the sensor has been updated
    bool dataUpdated{false};
};

/// \brief  Definition of the private data class
class simulation::SensorPowerSystemPrivate
{
    /// \brief Callback executed when a sensor is activated
    /// \param[in] _id The id of the sensor
    /// \param[in] _msg The message containing the activation state
public:
    void OnActivateSensor(int _id, const ignition::msgs::Boolean &_msg);

    /// \brief Check if the battery has sufficient charge
    /// \param[in] _ecm The entity component manager
    /// \return True if the battery has sufficient charge
public:
    void HasSufficientBattery(const ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Model name
public:
    std::string modelName;

    /// \brief Model entity
public:
    gz::sim::Model model{ignition::gazebo::kNullEntity};

    /// \brief Ignition communication node
public:
    ignition::transport::Node node;

    /// \brief Sensors information
public:
    std::vector<SensorInfo> sensorsInfo;

    /// \brief Flag to check if the battery is initialized
public:
    bool batteriesInitialized{false};
};

/////////////////////////////////////////////////
SensorPowerSystemPlugin::SensorPowerSystemPlugin()
    : System(), dataPtr(std::make_unique<SensorPowerSystemPrivate>())
{
}

/////////////////////////////////////////////////
SensorPowerSystemPlugin::~SensorPowerSystemPlugin() = default;

/////////////////////////////////////////////////
void SensorPowerSystemPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                        const std::shared_ptr<const sdf::Element> &_sdf,
                                        ignition::gazebo::EntityComponentManager &_ecm,
                                        ignition::gazebo::EventManager &_eventMgr)
{
    // Store the pointer to the model this battery is under
    auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm))
    {
        ignerr << "SensorPowerSystemPlugin plugin should be attached to a model entity. "
               << "Failed to initialize." << std::endl;
        return;
    }

    this->dataPtr->model = model;
    this->dataPtr->modelName = model.Name(_ecm);

    // Read power load and battery name from the sensors

    // camera sensors
    int sensorCount = 0;

    _ecm.Each<ignition::gazebo::components::Camera>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Camera *_camera) -> bool
        {
            auto cameraName = _ecm.Component<ignition::gazebo::components::Name>(_entity);
            if (cameraName)
            {
                auto cameraPtr = _ecm.Component<ignition::gazebo::components::Camera>(_entity);
                auto camera = cameraPtr->Data().CameraSensor();
                auto parent = camera->Element()->GetParent();
                if (parent->HasElement("power_load") && parent->HasElement("battery_name"))
                {
                    SensorInfo sensorInfo;
                    sensorInfo.id = sensorCount;
                    sensorInfo.name = cameraName->Data();
                    sensorInfo.powerLoad = parent->Get<double>("power_load");
                    sensorInfo.batteryName = parent->Get<std::string>("battery_name");
                    sensorInfo.enableSensor = true;
                    sensorInfo.dataUpdated = false;
                    igndbg << "CAMERA: " << sensorInfo.name << " id: " << sensorInfo.id << std::endl;
                    igndbg << "CAMERA: " << sensorInfo.name << " Power: " << sensorInfo.powerLoad << std::endl;
                    igndbg << "CAMERA: " << sensorInfo.name << " Battery name: " << sensorInfo.batteryName << std::endl;
                    igndbg << "CAMERA: " << sensorInfo.name << " is enabled: " << sensorInfo.enableSensor << std::endl;
                    igndbg << "CAMERA: " << sensorInfo.name << " data updated: " << sensorInfo.dataUpdated << std::endl;
                    igndbg << "CAMERA id: " << sensorInfo.id << std::endl;
                    this->dataPtr->sensorsInfo.emplace_back(std::move(sensorInfo));
                    sensorCount++;
                }
            }
            return true;
        });

    // imu sensors
    _ecm.Each<ignition::gazebo::components::Imu>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Imu *_imu) -> bool
        {
            // get the imu name
            auto imuName = _ecm.Component<ignition::gazebo::components::Name>(_entity);
            if (imuName)
            {
                auto imuPtr = _ecm.Component<ignition::gazebo::components::Imu>(_entity);
                auto imu = imuPtr->Data().ImuSensor();
                auto parent = imu->Element()->GetParent();
                if (parent->HasElement("power_load") && parent->HasElement("battery_name"))
                {
                    SensorInfo sensorInfo;
                    sensorInfo.id = sensorCount;
                    sensorInfo.name = imuName->Data();
                    sensorInfo.powerLoad = parent->Get<double>("power_load");
                    sensorInfo.batteryName = parent->Get<std::string>("battery_name");
                    sensorInfo.enableSensor = true;
                    sensorInfo.dataUpdated = false;
                    igndbg << "IMU: " << sensorInfo.name << " id: " << sensorInfo.id << std::endl;
                    igndbg << "IMU: " << sensorInfo.name << " Power: " << sensorInfo.powerLoad << std::endl;
                    igndbg << "IMU: " << sensorInfo.name << " Battery name: " << sensorInfo.batteryName << std::endl;
                    igndbg << "IMU: " << sensorInfo.name << " is enabled: " << sensorInfo.enableSensor << std::endl;
                    igndbg << "IMU: " << sensorInfo.name << " data updated: " << sensorInfo.dataUpdated << std::endl;
                    this->dataPtr->sensorsInfo.emplace_back(std::move(sensorInfo));
                    sensorCount++;
                }
            }
            return true;
        });

    // create a subscription to the sensor topic
    for (auto &sensor : this->dataPtr->sensorsInfo)
    {
        std::string stateTopic{"/model/" + this->dataPtr->model.Name(_ecm) + "/sensor/" + sensor.name + "/activate"};
        auto validSensorTopic = ignition::transport::TopicUtils::AsValidTopic(stateTopic);
        if (validSensorTopic.empty())
        {
            ignerr << "Failed to create valid topic. Not valid: ["
                   << sensor.name << "]" << std::endl;
            return;
        }
        std::function<void(const ignition::msgs::Boolean &)> callback = std::bind(&SensorPowerSystemPrivate::OnActivateSensor,
                                                                                  this->dataPtr.get(), sensor.id, std::placeholders::_1);
        this->dataPtr->node.Subscribe(validSensorTopic, callback);
    }
}

//////////////////////////////////////////////////
void SensorPowerSystemPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                        ignition::gazebo::EntityComponentManager &_ecm)

{
    if (_info.paused)
    {
        return;
    }
    if (!this->dataPtr->batteriesInitialized)
    {
        this->dataPtr->batteriesInitialized = true;
        _ecm.Each<ignition::gazebo::components::BatterySoC, ignition::gazebo::components::Name>(
            [&](const ignition::gazebo::Entity &_entity,
                const ignition::gazebo::components::BatterySoC *_batterySoc,
                const ignition::gazebo::components::Name *_name) -> bool
            {
                if (_name)
                {
                    for (auto &sensor : this->dataPtr->sensorsInfo)
                    {
                        if (sensor.batteryName == _name->Data())
                        {
                            igndbg << "Battery found for sensor: " << sensor.name << std::endl;
                            igndbg << "Battery name: " << _name->Data() << std::endl;
                            sensor.batteryExist = true;
                            sensor.batteryEntity = _entity;
                        }
                    }
                }
                return true;
            });
        for (auto &sensor : this->dataPtr->sensorsInfo)
        {
            if (sensor.batteryExist)
            {
                sensor.batteryConsumerEntity = _ecm.CreateEntity();
                ignition::gazebo::components::BatteryPowerLoadInfo batteryPowerLoad{
                    sensor.batteryEntity, sensor.powerLoad};
                _ecm.CreateComponent(sensor.batteryConsumerEntity, ignition::gazebo::components::BatteryPowerLoad(batteryPowerLoad));
                _ecm.SetParentEntity(sensor.batteryConsumerEntity, sensor.batteryEntity);
            }
            else
            {
                igndbg << "Sensor " << sensor.name << " battery does not exist" << std::endl;
            }
        }
    }
    else
    {
        for (auto &sensor : this->dataPtr->sensorsInfo)
        {
            if (sensor.batteryExist && sensor.hasEnoughBattery && sensor.dataUpdated)
            {
                float setPower = sensor.powerLoad;
                if (!sensor.enableSensor)
                {
                    setPower = 0.0;
                }
                std::lock_guard<std::mutex> lock(*sensor.mutex_ptr);
                sensor.dataUpdated = false;
                ignition::gazebo::v6::components::BatteryPowerLoadInfo batteryPowerLoad{
                    sensor.batteryConsumerEntity, setPower};
                _ecm.SetComponentData<ignition::gazebo::components::BatteryPowerLoad>(sensor.batteryConsumerEntity, batteryPowerLoad);
            }
        }
    }
}

/////////////////////////////////////////////////
void SensorPowerSystemPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                         const ignition::gazebo::EntityComponentManager &_ecm)

{
    if(_info.paused)
    {
        return;
    }
    this->dataPtr->HasSufficientBattery(_ecm);
}

/////////////////////////////////////////////////
void SensorPowerSystemPrivate::HasSufficientBattery(
    const ignition::gazebo::EntityComponentManager &_ecm)
{
      _ecm.Each<ignition::gazebo::components::BatterySoC>([&](
        const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::BatterySoC *_data
      ){
        auto BatteryName = _ecm.Component<ignition::gazebo::components::Name>(_entity);
        if(!BatteryName)
        {
            return false;
        }
        for(auto &sensor : this->sensorsInfo)
        {
            if(sensor.batteryName == BatteryName->Data())
            {
                if(_data->Data() <= 0.0)
                {
                sensor.hasEnoughBattery = false;
                }
                else 
                {
                sensor.hasEnoughBattery = true;
                }
            }
        }
        return true;
    });
}

/////////////////////////////////////////////////
void SensorPowerSystemPrivate::OnActivateSensor(int _id, const ignition::msgs::Boolean &_msg)
{
    std::lock_guard<std::mutex> lock(*this->sensorsInfo[_id].mutex_ptr);
    this->sensorsInfo[_id].enableSensor = _msg.data();
    this->sensorsInfo[_id].dataUpdated = true;
}

#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(SensorPowerSystemPlugin,
                    ignition::gazebo::System,
                    SensorPowerSystemPlugin::ISystemConfigure,
                    SensorPowerSystemPlugin::ISystemPreUpdate,
                    SensorPowerSystemPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SensorPowerSystemPlugin, "simulation::SensorPowerSystemPlugin")
