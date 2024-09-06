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

#ifndef CONFIG_CLASSES_HH_
#define CONFIG_CLASSES_HH_

#include <cmath>

namespace simulation
{
    class BladeConfig
    {
        public:
            double a0; // lift-curve slope

            double c; // chord

            double R; // length of blade

            double th_tw; // blade twist slope

            double I_beta; // blade inertia

            double delta_0; // drag-curve zero point

            double delta_2; //drag-curve slope

            BladeConfig(double a0_, double c_, double R_, double th_tw_, double I_beta_)
                : a0(a0_), c(c_), R(R_), th_tw(th_tw_), I_beta(I_beta_)
            {
            }
    };

    class RotorConfig
    {
    public:
        BladeConfig blade;  // Blade configuration
        double K_beta;      // Hinge stiffness
        int N_b;            // number of blades
        double I_R;         // moment of inertia of rotor and transmission system
        bool ccw;

        RotorConfig(const BladeConfig& blade_, double K_beta_, int N_b_, bool ccw_)
            : blade(blade_), K_beta(K_beta_), N_b(N_b_), ccw(ccw_)
        {
        }

        double solidity() const // rotor solidity
        {
            return N_b*blade.c/(M_PI*blade.R);
        }

        double gamma(double rho) const
        {
            return rho*blade.c*blade.a0*std::pow(blade.R,4)/blade.I_beta;
        }

    };
}

#endif