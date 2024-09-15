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

#ifndef DATA_CLASSES_HH_
#define DATA_CLASSES_HH_
#include "Matrix.hpp"
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <cmath>

#define TOL 0.01

using namespace gz;
using namespace math;

namespace simulation
{
    /**
     * @brief Represents the pitch state of a rotor, including collective, longitudinal, and lateral pitch components.
     */
    class PitchState
    {
        private:
            double th_0; /**< Collective pitch angle. */
            double th_1c; /**< Longitudinal pitch angle. */
            double th_1s; /**< Lateral pitch angle. */
        public:
            /**
             * @brief Default constructor initializing pitch angles to zero.
             */
            PitchState()
            {
                th_0 = 0;
                th_1c = 0;
                th_1s = 0;
            }

            /** @brief Gets the collective pitch angle. */
            double _0() const { return th_0; };

            /** @brief Gets the longitudinal pitch angle. */
            double _1c() const { return th_1c; };

            /** @brief Gets the lateral pitch angle. */
            double _1s() const { return th_1s; };

            /** @brief Sets the collective pitch angle. */
            void _0(double collective) { th_0 = collective; };

            /** @brief Sets the longitudinal pitch angle. */
            void _1c(double longitudinalPitch) { th_1c = longitudinalPitch; };
            
            /** @brief Sets the lateral pitch angle. */
            void _1s(double lateralPitch) { th_1s = lateralPitch; };

            /**
             * @brief Computes the longitudinal pitch in the wind-hub frame.
             * @param psi_w Rotor sideslip angle in radians.
             * @return Longitudinal pitch component.
             */
            double _1cw(double psi_w) const
            {
                return std::cos(psi_w)*th_1c + std::sin(psi_w)*th_1s;
            }

            /**
             * @brief Computes the lateral pitch in the wind-hub frame.
             * @param psi_w Rotor sideslip angle in radians.
             * @return Lateral pitch component.
             */
            double _1sw(double psi_w) const
            {
                return -std::sin(psi_w)*th_1c + std::cos(psi_w)*th_1s;
            }

            /**
             * @brief Constructor with initialization.
             * @param th_0_ Collective pitch.
             * @param th_1c_ Longitudinal pitch.
             * @param th_1s_ Lateral pitch.
             */
            PitchState(double th_0_, double th_1c_, double th_1s_)
            {
                th_0 = th_0_;
                th_1c = th_1c_;
                th_1s = th_1s_;
            }

            /**
             * @brief Converts the pitch angles to a matrix representation.
             * @return Matrix of pitch angles.
             */
            Matrix vec() 
            { 
                std::vector<std::vector<double>> columnMatrix = {
                    {th_0}, 
                    {th_1c}, 
                    {th_1s}
                };
                return Matrix(columnMatrix);
            };
        
    };

    /**
     * @brief Represents the flapping state of rotor blades, including coning, longitudinal, and lateral flapping.
     */
    class FlappingState
    {
        private:
            double beta_0; /**< Coning angle. */
            double beta_1c; /**< Longitudinal flapping angle. */
            double beta_1s; /**< Lateral flapping angle. */

        public:
            /**
             * @brief Default constructor initializing flapping angles to zero.
             */
            FlappingState() 
            {
                beta_0 = 0;
                beta_1c = 0;
                beta_1s = 0;
            };

            /**
             * @brief Constructor with initialization.
             * @param beta_0_ Coning angle.
             * @param beta_1c_ Longitudinal flapping angle.
             * @param beta_1s_ Lateral flapping angle.
             */
            FlappingState(double beta_0_, double beta_1c_, double beta_1s_)
            {
                beta_0  = beta_0_;
                beta_1c = beta_1c_;
                beta_1s = beta_1s_;
            }

            /** @brief Gets the coning angle. */
            double _0() const { return beta_0; };

            /** @brief Gets the longitudinal flapping angle. */
            double _1c() const { return beta_1c; };

            /** @brief Gets the lateral flapping angle. */
            double _1s() const { return beta_1s; };

            /**
             * @brief Computes the longitudinal flapping in the wind-hub frame.
             * @param psi_w Rotor sideslip angle in radians.
             * @return Longitudinal flapping component.
             */
            double _1cw(double psi_w) const
            {
                return std::cos(psi_w)*beta_1c + std::sin(psi_w)*beta_1s;
            }

            /**
             * @brief Computes the lateral flapping in the wind-hub frame.
             * @param psi_w Rotor sideslip angle in radians.
             * @return Lateral flapping component.
             */
            double _1sw(double psi_w) const
            {
                return -std::sin(psi_w)*beta_1c + std::cos(psi_w)*beta_1s;
            }

            /**
             * @brief Converts the flapping angles to a matrix representation.
             * @return Matrix of flapping angles.
             */
            Matrix vec() 
            { 
                std::vector<std::vector<double>> columnMatrix = {
                    {beta_0}, 
                    {beta_1c}, 
                    {beta_1s}
                };
                return Matrix(columnMatrix);
            };
    };

    /**
     * @brief Represents the inflow state of a rotor, including uniform and harmonic inflow components.
     */
    class InflowState
    {
        private:
            double lambda_0; /**< Uniform inflow component. */
            double lambda_1c; /**< Longitudinal inflow harmonic. */
            double lambda_1s; /**< Lateral inflow harmonic. */
        
        public:
            /**
             * @brief Default constructor initializing inflow components to zero.
             */
            InflowState()
            {
                lambda_0 = 0;
                lambda_1c = 0;
                lambda_1s = 0;
            }

            /**
             * @brief Constructor with initialization.
             * @param lambda_0_ Uniform inflow.
             * @param lambda_1c_ Longitudinal inflow harmonic.
             * @param lambda_1s_ Lateral inflow harmonic.
             */
            InflowState(double lambda_0_, double lambda_1c_, double lambda_1s_)
            {
                lambda_0 =  lambda_0_;
                lambda_1c = lambda_1c_;
                lambda_1s = lambda_1s_;
            }

            /** @brief Gets the uniform inflow component. */
            double _0() const { return lambda_0; };

            /** @brief Gets the longitudinal inflow harmonic. */
            double _1c() const { return lambda_1c; };

            /** @brief Gets the lateral inflow harmonic. */
            double _1s() const { return lambda_1s; };

            /**
             * @brief Computes the longitudinal inflow harmonic in the wind-hub frame.
             * @param psi_w Rotor sideslip angle in radians.
             * @return Longitudinal inflow component.
             */
            double _1cw(double psi_w) const
            {
                return std::cos(psi_w)*lambda_1c + std::sin(psi_w)*lambda_1s;
            }

            /**
             * @brief Computes the lateral inflow harmonic in the wind-hub frame.
             * @param psi_w Rotor sideslip angle in radians.
             * @return Lateral inflow component.
             */
            double _1sw(double psi_w) const
            {
                return -std::sin(psi_w)*lambda_1c + std::cos(psi_w)*lambda_1s;
            }

            /**
             * @brief Converts the inflow components to a matrix representation.
             * @return Matrix of inflow components.
             */
            Matrix vec() 
            { 
                std::vector<std::vector<double>> columnMatrix = {
                    {lambda_0}, 
                    {lambda_1c}, 
                    {lambda_1s}
                };
                return Matrix(columnMatrix);
            };
    };

    /**
     * @brief Represents the state of the body in different reference frames.
     * 
     * This class holds the velocity and angular velocity vectors, as well as their derivatives, 
     * in different frames of reference: link frame, hub frame, and wind frame.
     */
    class BodyState
    {
        public: // Input from Gazebo simulation

        Vector3d u_L; ///< Velocity vector in the link frame.
        Vector3d uDot_L; ///< Acceleration vector in the link frame.
        Vector3d om_L; ///< Angular velocity vector in the link frame.
        Vector3d omDot_L; ///< Angular acceleration vector in the link frame.

        Vector3d u_H; ///< Velocity vector in the hub frame.
        Vector3d uDot_H; ///< Acceleration vector in the hub frame.
        Vector3d om_H; ///< Angular velocity vector in the hub frame.
        Vector3d omDot_H; ///< Angular acceleration vector in the hub frame.

        Pose3d pose_HL_; ///< Pose of the hub relative to the link frame.
        Pose3d pose_WH_; ///< Pose of the wind frame relative to the hub frame.
        Matrix Delta_HW; ///< Transformation matrix from hub to wind frame.

        Vector3d u_W; ///< Velocity vector in the wind frame.
        Vector3d om_W; ///< Angular velocity vector in the wind frame.
        Vector3d omDot_W; ///< Angular acceleration vector in the wind frame.

        public:

            /**
             * @brief Default constructor for BodyState.
             */
            BodyState() {};

            /**
             * @brief Constructs a BodyState object with the given parameters.
             * 
             * Initializes the state vectors in the hub and wind frames based on the given
             * parameters in the link frame and the poses provided.
             * 
             * @param u_L_ Velocity vector in the link frame.
             * @param uDot_L_ Acceleration vector in the link frame.
             * @param om_L_ Angular velocity vector in the link frame.
             * @param omDot_L_ Angular acceleration vector in the link frame.
             * @param pose_HL_ Pose of the hub relative to the link frame.
             */
            BodyState(Vector3d u_L_, Vector3d uDot_L_, Vector3d om_L_, Vector3d omDot_L_, Pose3d pose_HL_)
            {
                // link frame variables
                u_L = u_L_;
                uDot_L = uDot_L_;
                om_L = om_L_;
                omDot_L = omDot_L_;

                // hub frame variables
                auto q_HL = pose_HL_.Rot();
                u_H = q_HL.RotateVector(u_L);
                uDot_H = q_HL.RotateVector(uDot_L);
                om_H = q_HL.RotateVector(om_L);
                omDot_H = q_HL.RotateVector(omDot_L);

                double psi = psi_w();
                auto q_WH = Quaterniond(Vector3d(0, 0, 1), psi);

                // from hub to wind frame
                u_W = q_WH.RotateVector(u_H);
                om_W = q_WH.RotateVector(om_H) + Vector3d(0, 0, psiDot_w());
                omDot_W = q_WH.RotateVector(omDot_H); // weak passage (third component is slightly incorrect, should add psiDoubleDot)
            }

            /**
             * @brief Computes the angular acceleration vector in the wind frame.
             * 
             * @param Omega Rotor angular velocity.
             * @return Angular acceleration vector in wind frame.
             */
            Vector3d omBarPrime_W(double Omega) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return omDot_W;
                }
                return omDot_W/std::pow(Omega,2);
            }

            /**
             * @brief Computes the angular velocity vector in the wind frame.
             * 
             * @param Omega Rotor angular velocity.
             * @return Angular velocity vector in the wind frame.
             */
            Vector3d omBar_W(double Omega) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return om_W;
                }
                return om_W/Omega;
            }

            /**
             * @brief Computes the angular velocity vector in the hub frame.
             * 
             * @param Omega Rotor angular velocity.
             * @return Angular velocity vector in the hub frame.
             */
            Vector3d omBar_H(double Omega) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return om_H;
                }
                return om_H/Omega;
            }

            /**
             * @brief Computes rotor sideslip angle.
             * 
             * @return Rotor sideslip angle in radians.
             */
            double psi_w() const
            {
                double u = u_H.X();
                double v = u_H.Y();
                return std::atan2(v,u);
            }

            /**
             * @brief Computes rate of change of rotor sideslip angle.
             * 
             * @return Rate of change of rotor sideslip angle in radians.
             */
            double psiDot_w()
            {
                double u = u_H.X();
                double v = u_H.Y();
                double uDot = uDot_H.X();
                double vDot = uDot_H.Y();

                return (vDot/u - v*uDot/std::pow(u,2))/(std::pow(v/u,2) + 1);
            }

            /**
             * @brief Computes the advance ratio of the rotor.
             * 
             * @param Omega Rotor angular velocity.
             * @param R Rotor radius.
             * @return Advance ratio.
             */
            double mu(double Omega, double R) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return u_W.X()/R;
                }
                return u_W.X()/(Omega*R);
            }

            /**
             * @brief Computes the advance ratio in the vertical direction.
             * 
             * @param Omega Rotor angular velocity.
             * @param R Rotor radius.
             * @return Vertical advance ratio.
             */
            double mu_z(double Omega, double R) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return u_W.Z()/R;
                }
                return u_W.Z()/(Omega*R);
            }
    };

    /**
     * @brief Represents the force and torque states of the blade.
     * 
     * This class holds the forces and torques applied to the blade in the hub frame, along with 
     * additional cached parameters related to blade forces.
     */
    class BladeForceState
    {

        public:
            Vector3d Forces_H; ///< Forces applied to the blade in the hub frame.
            Vector3d Torques_H; ///< Torques applied to the blade in the hub frame.

            /**
             * @brief Default constructor for BladeForceState.
             */
            BladeForceState()
            {
                Forces_H = Vector3d::Zero;
                Torques_H = Vector3d::Zero;
            }

        public: // cache
            double C_T; ///< Thrust coefficient.
            double F_1s_outPlane; ///< Out-of-plane force component.
            double F_1c_outPlane; ///< Out-of-plane force component.
        
        public:

            /**
             * @brief Constructs a BladeForceState object with the given forces and torques.
             * 
             * @param Forces_H_ Forces applied to the blade in the hub frame.
             * @param Torques_H_ Torques applied to the blade in the hub frame.
             */
            BladeForceState(Vector3d Forces_H_, Vector3d Torques_H_)
            {
                Forces_H = Forces_H_;
                Torques_H = Torques_H_;
            }
    };

    /**
     * @brief Represents the state of the rotor shaft.
     * 
     * This class holds the rotor's angular velocity and its derivative, 
     * as well as methods to compute related parameters.
     */
    class RotorShaftState
    {

        private: 
            double Omega_; ///< Rotor angular velocity.

        public:
            /**
             * @brief Gets the rotor's angular velocity.
             * 
             * @return Rotor angular velocity.
             */
            double Omega() const { return Omega_; };

            /**
             * @brief Sets the rotor's angular velocity.
             * 
             * @param val New rotor angular velocity.
             */
            void Omega(double val)
            {
                Omega_ = val;
            }

            double OmegaDot; ///< Derivative of the rotor's angular velocity.

            
            /**
             * @brief Default constructor for RotorShaftState.
             */
            RotorShaftState()
            {
                Omega_ = 0;
                OmegaDot = 0;
            }

            /**
             * @brief Computes the adimensional derivative of the angular velocity (wrt azimuth angle).
             * 
             * @return Adimensional derivative of Omega.
             */
            double OmegaBarPrime() const
            {
                return OmegaDot/(Omega_*Omega_);
            }

    };
}



#endif