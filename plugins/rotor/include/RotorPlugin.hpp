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
    /**
     * @brief RotorPlugin class is a plugin for simulating rotor dynamics in a Gazebo simulation.
     * This class implements the System interface and handles rotor physics, including blade flapping,
     * force generation, and inflow models.
     */
    class RotorPlugin
        : public System,
            public ISystemConfigure,
            public ISystemPreUpdate
    {
        public: 
            /** @brief Constructor for RotorPlugin */
            RotorPlugin() : rotorConfig(nullptr) {};

            /** @brief Destructor for RotorPlugin */
            ~RotorPlugin() override = default;

        /**
         * @brief Configure the plugin by setting up entities and initializing parameters.
         * @param[in] _entity The entity representing the rotor model.
         * @param[in] _sdf Shared pointer to the SDF element that defines the plugin.
         * @param[in] _ecm Entity Component Manager used to interact with the simulation.
         * @param[in] _eventMgr Event Manager to handle simulation events.
         */
        public: void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &_eventMgr) override;

        /**
         * @brief Called before each simulation step to update the state of the rotor.
         * @param[in] _info Update information, including simulation time and delta time.
         * @param[in] _ecm Entity Component Manager used to interact with the simulation.
         */
        public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

        private:

            /** @brief Pointer to the blade flapping model */
            std::shared_ptr<BladeFlappingModel> bladeFlappingModel;

            /** @brief Pointer to the blade force model */
            std::shared_ptr<BladeForceModel> bladeForceModel;

            /** @brief Pointer to the inflow model */
            std::shared_ptr<InflowModel> inflowModel;

        /** @brief Model representing the robot to which the rotor belongs */
        public: Model robot{kNullEntity};

        /** @brief Configuration data for the rotor (owns the pointer) */
        private: std::unique_ptr<RotorConfig> rotorConfig;  // Owning nullable pointer

        /** @brief Relative pose hub-link */
        private: gz::math::Pose3d pose_HL;

        /** @brief State of the rotor pitch */
        private: PitchState pitchState;

        /** @brief State of the rotor shaft */
        private: RotorShaftState shaftState;

        /** @brief State of the rotor body */
        private: BodyState bodyState;

        /** @brief Air density in the world */
        private: double worldDensity;

        /**
         * @brief Update the air density based on the rotor's height.
         * @param[in] height The height of the rotor above the ground.
         * @return True if the density was updated successfully, false otherwise.
         */
        private: bool updateDensityFromWorld(double height);

        private:
            /** @brief Link associated with robot main physical body (usually fuselage) */
            Link link;

            /** @brief Name of the rotor's link */
            std::string linkName;

            /** @brief Joint representing the rotor's physical attachment point */
            Joint joint;

            /** @brief Name of the rotor's joint */
            std::string jointName;

        private:

            /** @brief Transport node for communication */
            gz::transport::Node node;

            /** @brief Publisher for collective pitch values */
            gz::transport::Node::Publisher pub_coll;

            /** @brief Publisher for longitudinal cyclic pitch values */
            gz::transport::Node::Publisher pub_long_cycl;

            /** @brief Publisher for lateral cyclic pitch values */
            gz::transport::Node::Publisher pub_lat_cycl;

            /** @brief Publisher for angular velocity values */
            gz::transport::Node::Publisher pub_ang_vel;

            /**
             * @brief Set the collective pitch of the rotor.
             * @param[in] _msg Message containing the new collective pitch value.
             */
            void setCollectivePitch(const gz::msgs::Double &_msg);

            /**
             * @brief Set the longitudinal cyclic pitch of the rotor.
             * @param[in] longitudinalCyclicPitch The new longitudinal cyclic pitch value.
             */
            void setLongitudinalCyclicPitch(const gz::msgs::Double &longitudinalCyclicPitch);

            /**
             * @brief Set the lateral cyclic pitch of the rotor.
             * @param[in] lateralCyclicPitch The new lateral cyclic pitch value.
             */
            void setLateralCyclicPitch(const gz::msgs::Double &lateralCyclicPitch);

            /**
             * @brief Set the angular velocity of the rotor.
             * @param[in] angularVelocity The new angular velocity value.
             */
            void setAngularVelocity(const gz::msgs::Double &angularVelocity);

            /**
             * @brief Set the angular acceleration of the rotor.
             * @param[in] angularAcceleration The new angular acceleration value.
             */
            void setAngularAcceleration(const gz::msgs::Double &angularAcceleration);


    };
}



#endif // ROTOR_PLUGIN_HH_