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

#ifndef ATMOSPHERE_PLUGIN_HH_
#define ATMOSPHERE_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

#include <gz/math/Pose3.hh>
#include <memory>
#include "AtmosphereModel.hpp"

using namespace gz;
using namespace sim;

namespace simulation
{
    /**
     * @class AtmospherePlugin
     * @brief A plugin for simulating atmospheric effects within a simulation environment.
     *
     * This plugin integrates with the simulation environment, provides functionalities
     * related to atmosphere modeling and publishes data via services.
     */
    class AtmospherePlugin
        : public System,
            public ISystemConfigure
    {
        /**
         * @brief Default constructor.
         */
        public: AtmospherePlugin() {}

        /**
         * @brief Default destructor.
         */
        public: ~AtmospherePlugin() override = default;

        /**
         * @brief Configures the plugin with the given parameters.
         * @param _entity The entity to configure.
         * @param _sdf The SDF element providing configuration parameters.
         * @param _ecm The entity-component manager.
         * @param _eventMgr The event manager.
         */
        public: void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &_eventMgr) override;

        private:

            gz::transport::Node node; ///< Transport node for communication.

            std::shared_ptr<AtmosphereModel> atmosphereModel; ///< Shared pointer to the atmosphere model.

            /**
             * @brief Service callback to handle density requests.
             * @param _req The request message containing an height value.
             * @param _rep The response message containing the computed density.
             * @return True if the service request was successfully handled, false otherwise.
             */
            bool densityService(const gz::msgs::Double &_req, gz::msgs::Double &_rep);

    };

}

#endif // ATMOSPHERE_PLUGIN_HH_