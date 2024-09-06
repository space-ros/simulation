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

#ifndef BLADE_FLAPPING_MODEL_HH_
#define BLADE_FLAPPING_MODEL_HH_



#include "Matrix.hpp"
#include "ConfigClasses.hpp"
#include "DataClasses.hpp"

namespace simulation
{
    class BladeFlappingModel
    {
        public:

            explicit BladeFlappingModel(RotorConfig* config)
            : rotor(config) {}

            virtual ~BladeFlappingModel() = default; // Virtual destructor
            
            RotorConfig* rotor;

            FlappingState flappingState;

            virtual FlappingState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, double Omega, double rho) {return FlappingState(); };


    };

    class CenterSpringModel : public BladeFlappingModel
    {
        public:
            CenterSpringModel(RotorConfig* config)
                : BladeFlappingModel(config)  // Call base class constructor
            {
                // Additional initialization (if any)
                flappingState = FlappingState();
            }
        private:
            Matrix A_beta_th = Matrix(3, 4, 0.0);

            Matrix A_beta_lmd = Matrix(3, 3, 0.0);

            Matrix A_beta_om = Matrix(3, 4, 0.0);

        public: FlappingState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, double Omega, double rho) override;

        public: void updateModelMatrices(double rho, double mu, double Omega);

        private: Matrix inflowState2Vector(const InflowState& inflowState, double psi_w, double mu_z);

        private: Matrix pitchState2Vector(const PitchState& pitchState, double psi_w);

        private: Matrix bodyState2Vector(const BodyState& bodyState, double Omega);

    };


    class TalbotFlappingModel : public BladeFlappingModel
    {
        public:
            TalbotFlappingModel(RotorConfig* config)
                : BladeFlappingModel(config)  // Call base class constructor
            {
                // Additional initialization (if any)
                flappingState = FlappingState();
            }
        private:
            Matrix A_beta_th = Matrix(3, 4, 0.0);

            Matrix A_beta_lmd = Matrix(3, 3, 0.0);

            Matrix A_beta_om = Matrix(3, 4, 0.0);

        public: FlappingState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, double Omega, double rho) override;

        public: void updateModelMatrices(double rho, double mu, double Omega);

        private: Matrix inflowState2Vector(const InflowState& inflowState, double psi_w, double mu_z);

        private: Matrix pitchState2Vector(const PitchState& pitchState, double psi_w);

        private: Matrix bodyState2Vector(const BodyState& bodyState, double Omega);

    };


    class GordonFlappingModel : public BladeFlappingModel
    {
        public:
            GordonFlappingModel(RotorConfig* config)
                : BladeFlappingModel(config)  // Call base class constructor
            {
                // Additional initialization (if any)
                flappingState = FlappingState();
            }

        public: FlappingState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, double Omega, double rho) override;

    };
}


#endif