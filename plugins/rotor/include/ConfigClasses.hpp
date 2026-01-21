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
    /**
     * @brief Configuration parameters for an individual blade.
     *
     * This class holds the physical parameters for a rotor blade such as
     * its lift-curve slope, chord length, blade radius, twist, and inertia.
     */
    class BladeConfig
    {
        public:
            double a0; ///< Lift-curve slope (unit: 1/rad).
            double c;  ///< Blade chord length (unit: meters).
            double R;  ///< Blade radius (unit: meters).
            double th_tw; ///< Total blade twist (unit: radians).
            double I_beta; ///< Blade inertia about the hinge (unit: kg*m^2).
            double delta_0; ///< Drag-curve zero point.
            double delta_2; ///< Drag-curve slope.

            /**
             * @brief Constructor for BladeConfig.
             * 
             * @param a0_ Lift-curve slope.
             * @param c_ Blade chord length.
             * @param R_ Blade radius.
             * @param th_tw_ Total blade twist.
             * @param I_beta_ Blade inertia.
             */
            BladeConfig(double a0_, double c_, double R_, double th_tw_, double I_beta_)
                : a0(a0_), c(c_), R(R_), th_tw(th_tw_), I_beta(I_beta_)
            {
            }
    };

    /**
     * @brief Configuration parameters for a rotor system.
     *
     * This class stores the parameters for a rotor system, including
     * blade configuration, hinge stiffness, number of blades, and rotational inertia.
     */
    class RotorConfig
    {
    public:
        BladeConfig blade;  ///< Configuration of the individual blades.
        double K_beta;      ///< Rotor hinge stiffness (unit: N*m/rad).
        int N_b;            ///< Number of blades in the rotor.
        double I_R;         ///< Moment of inertia of the rotor and transmission system (unit: kg*m^2).
        bool ccw;           ///< Counter-clockwise rotation flag (true if the rotor rotates counter-clockwise).

        /**
         * @brief Constructor for RotorConfig.
         * 
         * @param blade_ Blade configuration.
         * @param K_beta_ Rotor hinge stiffness.
         * @param N_b_ Number of blades in the rotor.
         * @param ccw_ Counter-clockwise rotation flag.
         */
        RotorConfig(const BladeConfig& blade_, double K_beta_, int N_b_, bool ccw_)
            : blade(blade_), K_beta(K_beta_), N_b(N_b_), ccw(ccw_)
        {
        }

        /**
         * @brief Calculate the rotor solidity.
         * 
         * Rotor solidity is defined as the ratio of the total blade area to the rotor disk area.
         * 
         * @return Solidity of the rotor (unit: dimensionless).
         */
        double solidity() const // rotor solidity
        {
            return N_b*blade.c/(M_PI*blade.R);
        }

        /**
         * @brief Calculate the Lock number (gamma).
         * 
         * The Lock number is a dimensionless parameter that describes the ratio of aerodynamic to inertial forces acting on the blades.
         * 
         * @param rho Air density (unit: kg/m^3).
         * @return The Lock number (unit: dimensionless).
         */
        double gamma(double rho) const
        {
            return rho*blade.c*blade.a0*std::pow(blade.R,4)/blade.I_beta;
        }

    };
}

#endif // CONFIG_CLASSES_HH_