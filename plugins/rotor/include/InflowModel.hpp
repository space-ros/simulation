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

#ifndef INFLOW_MODEL_HH_
#define INFLOW_MODEL_HH_

#include "ConfigClasses.hpp"
#include "DataClasses.hpp"
#include <vector>

namespace simulation
{
    /**
     * @brief Base class for inflow models used in rotor simulations.
     * This class defines the interface for different types of inflow models
     * that compute the state of inflow based on rotor and body dynamics.
     */
    class InflowModel
    {
        public:
            /**
             * @brief Constructor for the InflowModel class.
             * @param config Pointer to the rotor configuration object.
             */
            explicit InflowModel(RotorConfig* config)
            : rotor(config) {}

            /** @brief Pointer to the rotor configuration. */
            RotorConfig* rotor;

            /** @brief The state of the inflow computed by the model. */
            InflowState inflowState;

            /**
             * @brief Virtual method to compute the inflow state.
             * This method should be overridden by derived classes to compute the inflow state
             * based on the pitch, flapping, blade force, and body state.
             * @param pitchState The state of the rotor's pitch.
             * @param flappingState The state of blade flapping.
             * @param bladeForceState The state of the blade forces.
             * @param bodyState The state of the body (e.g., helicopter or drone).
             * @param Omega The rotor's angular velocity.
             * @return The computed inflow state.
             */
            virtual InflowState computeState(const PitchState& pitchState, const FlappingState &flappingState, const BladeForceState &bladeForceState, const BodyState& bodyState, double Omega) { return InflowState(); };

    };

    /**
     * @brief Derived class that implements a harmonic inflow model.
     * This model computes inflow using harmonic expansion methods, which include
     * computing both uniform inflow and first-order harmonics.
     */
    class HarmonicInflowModel: public InflowModel
    {

        public:
            /**
             * @brief Constructor for the HarmonicInflowModel class.
             * @param config Pointer to the rotor configuration object.
             */
            HarmonicInflowModel(RotorConfig* config)
                : InflowModel(config)  // Call base class constructor
            {
                // Additional initialization (if any)
                inflowState = InflowState();
            }
        
        private:
            /**
             * @brief Computes the uniform inflow (lateral and longitudinal) for the rotor.
             * @param lmd_0 The base inflow ratio (non-dimensional).
             * @param C_T The thrust coefficient.
             * @param mu The advance ratio (forward flight speed / rotor tip speed).
             * @param mu_z The vertical advance ratio (vertical speed / rotor tip speed).
             * @return The computed uniform inflow.
             */
            double computeUniformInflow(double lmd_0, double C_T, double mu, double mu_z);

            /**
             * @brief Computes the harmonic components of the inflow.
             * @param pitchState The state of the rotor's pitch.
             * @param flappingState The state of blade flapping.
             * @param bodyState The state of the body (e.g., helicopter or drone).
             * @param Omega The rotor's angular velocity.
             * @param lmd_0 The base inflow ratio.
             * @return A vector containing the harmonic inflow components.
             */
            std::vector<double> computeHarmonicsInflow(const PitchState& pitchState, const FlappingState &flappingState, const BodyState& bodyState, double Omega, double lmd_0);
        
        public:

            /**
             * @brief Overrides the base computeState method to compute the inflow using harmonics.
             * This method computes both uniform inflow and harmonic inflow based on rotor
             * and body dynamics.
             * @param pitchState The state of the rotor's pitch.
             * @param flappingState The state of blade flapping.
             * @param bladeForceState The state of the blade forces.
             * @param bodyState The state of the body (e.g., helicopter or drone).
             * @param Omega The rotor's angular velocity.
             * @return The computed inflow state.
             */
            virtual InflowState computeState(const PitchState& pitchState, const FlappingState &flappingState, const BladeForceState &bladeForceState, const BodyState& bodyState, double Omega) override;
    };
}



#endif