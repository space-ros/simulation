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

#ifndef ATMOSPHERE_MODEL_HH_
#define ATMOSPHERE_MODEL_HH_


namespace simulation
{
    /**
     * @class AtmosphereModel
     * @brief Abstract base class for atmosphere models.
     *
     * This class defines the interface for different atmosphere models used in the simulation.
     */
    class AtmosphereModel
    {
        public:

            /**
             * @brief Virtual destructor for proper cleanup of derived classes.
             */
            virtual ~AtmosphereModel() = default; // Virtual destructor

            /**
             * @brief Pure virtual function to get the atmospheric density at a given height.
             * @param height The height at which to retrieve the atmospheric density.
             * @return The atmospheric density at the specified height.
             */
            virtual double getDensity(double height) = 0;

    };

    /**
     * @class UniformDensityAtmosphereModel
     * @brief Concrete implementation of AtmosphereModel with uniform density.
     *
     * This class represents an atmosphere model where the density is constant regardless of height.
     */
    class UniformDensityAtmosphereModel : public AtmosphereModel
    {
        private : double rho_0; ///< Constant density value.

        public: 

            /**
             * @brief Constructor to initialize the uniform density atmosphere model.
             * @param rho_0_ The constant density to be used in the model.
             */
            UniformDensityAtmosphereModel(double rho_0_) : rho_0(rho_0_) {};

            /**
             * @brief Get the density at a given height.
             * @param height The height at which to retrieve the atmospheric density.
             * @return The constant density value, rho_0, irrespective of height.
             */
            double getDensity(double height) override;
    };
}

#endif // ATMOSPHERE_MODEL_HH_