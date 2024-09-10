/*
 * Copyright (C) 2024 Davide Zamblera
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

#include "RotorPlugin.hpp"

#include <gz/common/Profiler.hh>
#include <gz/common/Console.hh>

#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/Pose.hh"
#include <memory>
#include <exception>
#include <thread>
#include <cmath>

#define DEFAULT_WORLD_DENSITY 0.02
#define ANIM_SPEED 20.0
#define EFFORT_VALID_LIMIT 10000.0
using namespace gz;
using namespace sim;
using namespace common;
using namespace math;
using namespace simulation;


template <typename T>
T SafeGetElement(const std::shared_ptr<const sdf::Element> &element, 
                 const std::string &elementName)
{
    if (element->HasElement(elementName))
    {
        ignmsg << "SDF Element: " << elementName << " retrieved correctly." << std::endl;
        return element->Get<T>(elementName);
    }
    else
    {
        throw std::runtime_error("Failure to retrieve SDF element " + elementName);
    }
}

bool RotorPlugin::updateDensityFromWorld(double height)
{
    // Prepare the input parameters.
    gz::msgs::Double req;
    req.set_data(height);

    gz::msgs::Double rep;
    bool result;
    unsigned int timeout = 100;

    bool executed = node.Request("/atmo/density", req, timeout, rep, result);

    if (executed)
    {
        if (result)
        {
            worldDensity = rep.data();
            return true;
        }
    }
    return false;
}


void RotorPlugin::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm, EventManager &_eventMgr)
{
    auto sdfClone = _sdf->Clone();
    linkName = SafeGetElement<std::string>(sdfClone, "link_name");
    jointName = SafeGetElement<std::string>(sdfClone, "joint_name"); 
    std::string col_topic = jointName + "/collective";

    // Subscribe to input topics
    pub_coll = node.Advertise<gz::msgs::Double>(col_topic);
    if (!node.Subscribe(col_topic, &RotorPlugin::setCollectivePitch, this))
    {
        ignerr << "Error subscribing to topic [" << col_topic << "]" << std::endl;
        return;
    }

    std::string lon_topic = jointName + "/longitudinal_cyclic";
    pub_long_cycl = node.Advertise<gz::msgs::Double>(lon_topic);
    if (!node.Subscribe(lon_topic, &RotorPlugin::setLongitudinalCyclicPitch, this))
    {
        ignerr << "Error subscribing to topic [" << lon_topic << "]" << std::endl;
        return;
    }

    std::string lat_topic = jointName + "/lateral_cyclic";
    pub_lat_cycl = node.Advertise<gz::msgs::Double>(lat_topic);
    if (!node.Subscribe(lat_topic, &RotorPlugin::setLateralCyclicPitch, this))
    {
        ignerr << "Error subscribing to topic [" << lat_topic << "]" << std::endl;
        return;
    }

    std::string ang_vel_topic = jointName + "/angular_velocity";
    pub_ang_vel = node.Advertise<gz::msgs::Double>(ang_vel_topic);
    if (!node.Subscribe(ang_vel_topic, &RotorPlugin::setAngularVelocity, this))
    {
        ignerr << "Error subscribing to topic [" << ang_vel_topic << "]" << std::endl;
        return;
    }

    // Obtain parameters from sdf configuration elements
    auto configElem = sdfClone->GetElement("config");
    auto bladeElem = configElem->GetElement("blade");

    double a0_ = SafeGetElement<double>(bladeElem, "lift_slope");
    double R_ = SafeGetElement<double>(bladeElem, "radius");
    double c_ = SafeGetElement<double>(bladeElem, "chord");
    double delta_2_ = SafeGetElement<double>(bladeElem, "drag_quadric");
    double delta_0_ = SafeGetElement<double>(bladeElem, "drag_constant");
    double I_beta_ = SafeGetElement<double>(bladeElem, "blade_inertia");
    double th_tw_ = SafeGetElement<double>(bladeElem, "twist_slope");

    double K_beta_ = SafeGetElement<double>(configElem, "hinge_stiffness");
    int N_b_ = SafeGetElement<int>(configElem, "num_blades");
    double I_R_ = SafeGetElement<double>(configElem, "rotor_inertia");
    bool ccw_ = true;
    if (configElem->HasElement("direction"))
    {
        std::string val = configElem->Get<std::string>("direction");
        if (val == "ccw")
        {
            ccw_ = true;
        }
        else if (val == "cw")
        {
            ccw_ = false;
        }
        else
        {
            std::runtime_error("Failure to retrieve SDF element direction");
        }
    }

    pose_HL = SafeGetElement<Pose3d>(sdfClone, "link_to_hub_frame");

    robot = Model(_entity);

    if (!linkName.empty() && robot.Valid(_ecm))
    {
        auto tmpEntity = robot.LinkByName(_ecm, linkName);
        if (tmpEntity != kNullEntity)
        {
            link = Link(tmpEntity);
            link.EnableVelocityChecks(_ecm, true);
            link.EnableAccelerationChecks(_ecm, true);
        }
    }

    if (!jointName.empty() && robot.Valid(_ecm))
    {
        auto tmpEntity = robot.JointByName(_ecm, jointName);
        if (tmpEntity != kNullEntity)
        {
            joint = Joint(tmpEntity);
        }
    }

    auto bladeConfig = BladeConfig(a0_, c_, R_, th_tw_, I_beta_);
    rotorConfig = std::make_unique<RotorConfig>(bladeConfig, K_beta_, N_b_, ccw_);

    // Instantiate dynamic models, change the if-else statements to add new models 
    std::string flappingModelName = SafeGetElement<std::string>(sdfClone, "flapping_model");
    std::string forceModelName = SafeGetElement<std::string>(sdfClone, "force_model");
    std::string inflowModelName = SafeGetElement<std::string>(sdfClone, "inflow_model");

    if (flappingModelName == "center_spring")
    {
        bladeFlappingModel = std::make_shared<CenterSpringModel>(rotorConfig.get());
    }
    else if (flappingModelName == "gordon")
    {
        bladeFlappingModel = std::make_shared<GordonFlappingModel>(rotorConfig.get());
    }

    if (forceModelName == "base")
    {
        bladeForceModel = std::make_shared<BladeForceModel>(rotorConfig.get());
    }

    if (inflowModelName == "harmonic_1")
    {
        inflowModel = std::make_shared<HarmonicInflowModel>(rotorConfig.get());
    }

    worldDensity = DEFAULT_WORLD_DENSITY;
}

void RotorPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    
    if (_info.paused)
    {
        // If paused, terminate the function early
        return;
    }

    if (shaftState.Omega() >= 0.0)
    {
        joint.SetVelocity(_ecm, {std::clamp(shaftState.Omega(), 0.0, ANIM_SPEED)});
    }
    auto entity = link.Entity();

    // Obtain motion and pose information from the link 
    auto linearVelComp_L = _ecm.Component<components::LinearVelocity>(entity);
    auto angularVelComp_L = _ecm.Component<components::AngularVelocity>(entity);
    auto linearAccComp_L = _ecm.Component<components::LinearAcceleration>(entity);
    auto angularAccComp_L = _ecm.Component<components::AngularAcceleration>(entity);
    auto poseComp_LG = _ecm.Component<components::WorldPose>(entity);
    Pose3d pose_LG;

    if (linearVelComp_L && angularVelComp_L && linearAccComp_L && angularAccComp_L && poseComp_LG)
    {
        auto u_L = linearVelComp_L->Data();
        auto omega_L = angularVelComp_L->Data();
        auto uDot_L = linearAccComp_L->Data();
        auto omegaDot_L = angularAccComp_L->Data();
        pose_LG = poseComp_LG->Data();


        bodyState = BodyState(u_L, uDot_L, omega_L, omegaDot_L, pose_HL);
    }
    else
    {
        return;
    }

    // current values of states (from swashplate system and shaft system plugin classes)
    PitchState currentPitchState = pitchState;
    RotorShaftState currentShaftState = shaftState;


    if (poseComp_LG)
    {
        double height = pose_LG.Pos().Z();
        bool result = updateDensityFromWorld(height);
        if (!result)
        {
            ignwarn << "Density not obtained from world, last known density will be used in the simulation" << std::endl;
        }
    }
    
    // compute states
    auto bladeForceState = bladeForceModel->computeState(currentPitchState, inflowModel->inflowState, bodyState, bladeFlappingModel->flappingState, currentShaftState, worldDensity);
    auto inflowState = inflowModel->computeState(currentPitchState, bladeFlappingModel->flappingState, bladeForceState, bodyState, currentShaftState.Omega());
    auto bladeFlappingState = bladeFlappingModel->computeState(currentPitchState, inflowModel->inflowState, bodyState, currentShaftState.Omega(), worldDensity);

    // update the states
    bladeForceModel->bladeForceState = bladeForceState;
    inflowModel->inflowState = inflowState;
    bladeFlappingModel->flappingState = bladeFlappingState;

    // apply forces to model
    Vector3d F_H = bladeForceState.Forces_H;
    Vector3d T_H = bladeForceState.Torques_H;

    // Limit forces
    if (F_H.Length() > EFFORT_VALID_LIMIT) {
        F_H.Normalize(); // Normalize the force vector
        F_H *= EFFORT_VALID_LIMIT; // Scale it to the maximum limit
    }
    F_H.Correct();

    // Limit torques
    if (T_H.Length() > EFFORT_VALID_LIMIT) {
        T_H.Normalize(); // Normalize the torque vector
        T_H *= EFFORT_VALID_LIMIT; // Scale it to the maximum limit
    }
    T_H.Correct();

    auto q_HL = pose_HL.Rot();
    auto r_HL_H = pose_HL.Pos();

    Vector3d F_L = q_HL.RotateVector(F_H);
    Vector3d T_L = q_HL.RotateVector(T_H + r_HL_H.Cross(F_H));

    auto q_LG = pose_LG.Rot();
    Vector3d F_G = q_LG.RotateVector(F_L);
    Vector3d T_G = q_LG.RotateVector(T_L);

    link.AddWorldWrench(_ecm, F_G, T_G);

}

void RotorPlugin::setCollectivePitch(const gz::msgs::Double &_msg)
{
    pitchState._0(_msg.data());
}

void RotorPlugin::setLongitudinalCyclicPitch(const gz::msgs::Double &longitudinalCyclicPitch)
{
    pitchState._1s(longitudinalCyclicPitch.data());
}

void RotorPlugin::setLateralCyclicPitch(const gz::msgs::Double &lateralCyclicPitch)
{
    pitchState._1c(lateralCyclicPitch.data());
}

void RotorPlugin::setAngularVelocity(const gz::msgs::Double &angularVelocity)
{
    if (angularVelocity.data() >= 0.0)
    {
        shaftState.Omega(angularVelocity.data());
    }
    else
    {
        ignwarn << "Set angular velocity callback for " << jointName << " failed, velocity must be >= 0.0" << std::endl;
    }
}

void RotorPlugin::setAngularAcceleration(const gz::msgs::Double &angularAcceleration)
{
    shaftState.OmegaDot = angularAcceleration.data();
}

IGNITION_ADD_PLUGIN(RotorPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
