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
    class AtmosphereModel
    {
        public:

            virtual ~AtmosphereModel() = default; // Virtual destructor

            virtual double getDensity(double height) = 0;

    };

    class UniformDensityAtmosphereModel : public AtmosphereModel
    {
        private : double rho_0;

        public: 

            UniformDensityAtmosphereModel(double rho_0_) : rho_0(rho_0_) {};

            double getDensity(double height) override;
    };
}

#endif