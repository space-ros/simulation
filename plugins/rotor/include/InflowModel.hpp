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
    class InflowModel
    {
        public:

            explicit InflowModel(RotorConfig* config)
            : rotor(config) {}

            RotorConfig* rotor;

            InflowState inflowState;

            virtual InflowState computeState(const PitchState& pitchState, const FlappingState &flappingState, const BladeForceState &bladeForceState, const BodyState& bodyState, double Omega) { return InflowState(); };

    };

    class HarmonicInflowModel: public InflowModel
    {

        public:
            HarmonicInflowModel(RotorConfig* config)
                : InflowModel(config)  // Call base class constructor
            {
                // Additional initialization (if any)
                inflowState = InflowState();
            }
        
        private:
            double computeUniformInflow(double lmd_0, double C_T, double mu, double mu_z);

            std::vector<double> computeHarmonicsInflow(const PitchState& pitchState, const FlappingState &flappingState, const BodyState& bodyState, double Omega, double lmd_0);
        
        public:
            virtual InflowState computeState(const PitchState& pitchState, const FlappingState &flappingState, const BladeForceState &bladeForceState, const BodyState& bodyState, double Omega) override;
    };
}



#endif