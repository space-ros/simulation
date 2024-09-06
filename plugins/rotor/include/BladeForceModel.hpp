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

#ifndef BLADE_FORCE_MODEL_HH_
#define BLADE_FORCE_MODEL_HH_

#include "ConfigClasses.hpp"
#include "DataClasses.hpp"

namespace simulation
{
    class BladeForceModel
    {
        
        public:
            RotorConfig* rotor;
            
            explicit BladeForceModel(RotorConfig* config)
            : rotor(config) {}

            virtual ~BladeForceModel() = default; // Virtual destructor

            BladeForceState bladeForceState;

            virtual BladeForceState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, const FlappingState &flappingState, const RotorShaftState &shaftState, double rho);

            
    };
}


#endif