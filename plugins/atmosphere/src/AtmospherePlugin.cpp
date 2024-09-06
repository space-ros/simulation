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

#include "AtmospherePlugin.hpp"
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <memory>
#include <exception>
#include <cmath>


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

bool AtmospherePlugin::densityService(const gz::msgs::Double &_req, gz::msgs::Double &_rep)
{
    double height = _req.data();

    double density = atmosphereModel->getDensity(height);

    _rep.set_data(density);

    return true;
}

void AtmospherePlugin::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm, EventManager &_eventMgr)
{
    auto sdfClone = _sdf->Clone();
    double rho_0_ = SafeGetElement<double>(sdfClone, "surface_density");
    std::string modelName = SafeGetElement<std::string>(sdfClone, "model");

    if (modelName == "uniform_density")
    {
        atmosphereModel = std::make_shared<UniformDensityAtmosphereModel>(rho_0_);
    }

    std::string service = "/atmo/density";

    // Advertise a service call.
    if (!node.Advertise(service, &AtmospherePlugin::densityService, this))
    {
        ignerr << "Error advertising service [" << service << "]" << std::endl;
        return;
    }

}

IGNITION_ADD_PLUGIN(AtmospherePlugin, gz::sim::System, gz::sim::ISystemConfigure)