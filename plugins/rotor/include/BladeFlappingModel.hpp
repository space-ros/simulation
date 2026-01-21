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
    /**
     * @brief Base class for blade flapping models.
     * 
     * This class provides the foundation for computing the flapping state of rotor blades
     * based on pitch, inflow, and body dynamics.
     */
    class BladeFlappingModel
    {
        public:

            /**
             * @brief Constructor for BladeFlappingModel.
             * 
             * @param config Pointer to a RotorConfig object that defines the rotor configuration.
             */
            explicit BladeFlappingModel(RotorConfig* config)
            : rotor(config) {}

            /**
             * @brief Virtual destructor.
             * 
             * Ensures proper cleanup in derived classes.
             */
            virtual ~BladeFlappingModel() = default; // Virtual destructor
            
            RotorConfig* rotor; ///< Pointer to the rotor configuration.

            FlappingState flappingState; ///< State of the blade flapping.

            /**
             * @brief Compute the blade flapping state.
             * 
             * Computes the state of the blade flapping based on the provided pitch, inflow, and body dynamics.
             * 
             * @param pitchState State of the blade pitch.
             * @param inflowState State of the inflow.
             * @param bodyState State of the body dynamics.
             * @param Omega Rotor angular velocity.
             * @param rho Air density (unit: kg/m^3).
             * 
             * @return FlappingState object representing the computed flapping state of the blades.
             */
            virtual FlappingState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, double Omega, double rho) {return FlappingState(); };


    };

    /**
     * @brief Center spring model for blade flapping dynamics.
     * 
     * This model computes the flapping state of the rotor blades based on the center spring dynamics.
     */
    class CenterSpringModel : public BladeFlappingModel
    {
        public:
            /**
             * @brief Constructor for CenterSpringModel.
             * 
             * @param config Pointer to a RotorConfig object that defines the rotor configuration.
             */
            CenterSpringModel(RotorConfig* config)
                : BladeFlappingModel(config)  // Call base class constructor
            {
                // Additional initialization (if any)
                flappingState = FlappingState();
            }
        private:
            Matrix A_beta_th = Matrix(3, 4, 0.0); ///< Matrix representing the pitch-flapping dynamics.

            Matrix A_beta_lmd = Matrix(3, 3, 0.0); ///< Matrix representing the inflow-flapping dynamics.

            Matrix A_beta_om = Matrix(3, 4, 0.0); ///< Matrix representing the body-flapping dynamics.

        /**
         * @brief Compute the flapping state based on center spring dynamics.
         * 
         * @param pitchState State of the blade pitch.
         * @param inflowState State of the inflow.
         * @param bodyState State of the body dynamics.
         * @param Omega Rotor angular velocity.
         * @param rho Air density (unit: kg/m^3).
         * 
         * @return FlappingState object representing the computed flapping state.
         */
        public: FlappingState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, double Omega, double rho) override;

        /**
         * @brief Update the model matrices based on the current conditions.
         * 
         * @param rho Air density (unit: kg/m^3).
         * @param mu Advance ratio.
         * @param Omega Rotor angular velocity.
         */
        public: void updateModelMatrices(double rho, double mu, double Omega);

        /**
         * @brief Convert inflow state to a vector representation.
         * 
         * @param inflowState State of the inflow.
         * @param psi_w Blade azimuth angle.
         * @param mu_z Axial advance ratio.
         * 
         * @return Matrix representing the inflow state.
         */
        private: Matrix inflowState2Vector(const InflowState& inflowState, double psi_w, double mu_z);

        /**
         * @brief Convert pitch state to a vector representation.
         * 
         * @param pitchState State of the blade pitch.
         * @param psi_w Blade azimuth angle.
         * 
         * @return Matrix representing the pitch state.
         */
        private: Matrix pitchState2Vector(const PitchState& pitchState, double psi_w);
        
        /**
         * @brief Convert body state to a vector representation.
         * 
         * @param bodyState State of the body dynamics.
         * @param Omega Rotor angular velocity.
         * 
         * @return Matrix representing the body state.
         */
        private: Matrix bodyState2Vector(const BodyState& bodyState, double Omega);

    };

    /**
     * @brief Talbot model for blade flapping dynamics.
     * 
     * This model computes the flapping state of the rotor blades using Talbot's method.
     */
    class TalbotFlappingModel : public BladeFlappingModel
    {
        public:
            /**
             * @brief Constructor for TalbotFlappingModel.
             * 
             * @param config Pointer to a RotorConfig object that defines the rotor configuration.
             */
            TalbotFlappingModel(RotorConfig* config)
                : BladeFlappingModel(config)
            {
                flappingState = FlappingState();
            }
        private:
            Matrix A_beta_th = Matrix(3, 4, 0.0); ///< Matrix representing the pitch-flapping dynamics.

            Matrix A_beta_lmd = Matrix(3, 3, 0.0); ///< Matrix representing the inflow-flapping dynamics.

            Matrix A_beta_om = Matrix(3, 4, 0.0); ///< Matrix representing the body-flapping dynamics.

        /**
         * @brief Compute the flapping state based on Talbot's method.
         * 
         * @param pitchState State of the blade pitch.
         * @param inflowState State of the inflow.
         * @param bodyState State of the body dynamics.
         * @param Omega Rotor angular velocity.
         * @param rho Air density (unit: kg/m^3).
         * 
         * @return FlappingState object representing the computed flapping state.
         */
        public: FlappingState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, double Omega, double rho) override;

        /**
         * @brief Update the model matrices based on the current conditions.
         * 
         * @param rho Air density (unit: kg/m^3).
         * @param mu Advance ratio.
         * @param Omega Rotor angular velocity.
         */
        public: void updateModelMatrices(double rho, double mu, double Omega);

        /**
         * @brief Convert inflow state to a vector representation.
         * 
         * @param inflowState State of the inflow.
         * @param psi_w Blade azimuth angle.
         * @param mu_z Axial advance ratio.
         * 
         * @return Matrix representing the inflow state.
         */
        private: Matrix inflowState2Vector(const InflowState& inflowState, double psi_w, double mu_z);

        /**
         * @brief Convert pitch state to a vector representation.
         * 
         * @param pitchState State of the blade pitch.
         * @param psi_w Blade azimuth angle.
         * 
         * @return Matrix representing the pitch state.
         */
        private: Matrix pitchState2Vector(const PitchState& pitchState, double psi_w);

        /**
         * @brief Convert body state to a vector representation.
         * 
         * @param bodyState State of the body dynamics.
         * @param Omega Rotor angular velocity.
         * 
         * @return Matrix representing the body state.
         */
        private: Matrix bodyState2Vector(const BodyState& bodyState, double Omega);

    };

    /**
     * @brief Gordon model for blade flapping dynamics.
     * 
     * This model computes the flapping state of the rotor blades using Gordon's method.
     */
    class GordonFlappingModel : public BladeFlappingModel
    {
        public:
            /**
             * @brief Constructor for GordonFlappingModel.
             * 
             * @param config Pointer to a RotorConfig object that defines the rotor configuration.
             */
            GordonFlappingModel(RotorConfig* config)
                : BladeFlappingModel(config)  // Call base class constructor
            {
                // Additional initialization (if any)
                flappingState = FlappingState();
            }

        /**
         * @brief Compute the flapping state based on Gordon's method.
         * 
         * @param pitchState State of the blade pitch.
         * @param inflowState State of the inflow.
         * @param bodyState State of the body dynamics.
         * @param Omega Rotor angular velocity.
         * @param rho Air density (unit: kg/m^3).
         * 
         * @return FlappingState object representing the computed flapping state.
         */
        public: FlappingState computeState(const PitchState& pitchState, const InflowState& inflowState, const BodyState& bodyState, double Omega, double rho) override;

    };
}


#endif // BLADE_FLAPPING_MODEL_HH_