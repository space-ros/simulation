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

#ifndef ROTOR_PLUGIN_HH_
#define ROTOR_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

#include <gz/math/Pose3.hh>
#include <memory>
#include "ConfigClasses.hpp"
#include "DataClasses.hpp"
#include "BladeFlappingModel.hpp"
#include "BladeForceModel.hpp"
#include "InflowModel.hpp"

using namespace gz;
using namespace sim;

namespace simulation
{
    class RotorPlugin
        : public System,
            public ISystemConfigure,
            public ISystemPreUpdate
    {
        public: RotorPlugin() : rotorConfig(nullptr) {};

        public: ~RotorPlugin() override = default;

        public: void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &_eventMgr) override;

        public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

        private:

                std::shared_ptr<BladeFlappingModel> bladeFlappingModel;

                std::shared_ptr<BladeForceModel> bladeForceModel;

                std::shared_ptr<InflowModel> inflowModel;

        public: Model robot{kNullEntity};

        private: std::unique_ptr<RotorConfig> rotorConfig;  // Owning nullable pointer

        private: gz::math::Pose3d pose_HL;

        private: PitchState pitchState;

        private: RotorShaftState shaftState;

        private: BodyState bodyState;

        private: double worldDensity;

        private: bool updateDensityFromWorld(double height);

        private:
            // link of the rotor
            Link link;

            std::string linkName;

            Joint joint;

            std::string jointName;

        private:

            gz::transport::Node node;

            gz::transport::Node::Publisher pub_coll;

            gz::transport::Node::Publisher pub_long_cycl;

            gz::transport::Node::Publisher pub_lat_cycl;

            gz::transport::Node::Publisher pub_ang_vel;

            void setCollectivePitch(const gz::msgs::Double &_msg);

            void setLongitudinalCyclicPitch(const gz::msgs::Double &longitudinalCyclicPitch);

            void setLateralCyclicPitch(const gz::msgs::Double &lateralCyclicPitch);

            void setAngularVelocity(const gz::msgs::Double &angularVelocity);

            void setAngularAcceleration(const gz::msgs::Double &angularAcceleration);


    };
}



#endif