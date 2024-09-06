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
    class PitchState
    {
        private:
            double th_0;
            double th_1c;
            double th_1s;

        public:
            PitchState()
            {
                th_0 = 0;
                th_1c = 0;
                th_1s = 0;
            }

            double _0() const { return th_0; };
            double _1c() const { return th_1c; };
            double _1s() const { return th_1s; };

            void _0(double collective) { th_0 = collective; };
            void _1c(double longitudinalPitch) { th_1c = longitudinalPitch; };
            void _1s(double lateralPitch) { th_1s = lateralPitch; };
            double _1cw(double psi_w) const
            {
                return std::cos(psi_w)*th_1c + std::sin(psi_w)*th_1s;
            }
            double _1sw(double psi_w) const
            {
                return -std::sin(psi_w)*th_1c + std::cos(psi_w)*th_1s;
            }
            PitchState(double th_0_, double th_1c_, double th_1s_)
            {
                th_0 = th_0_;
                th_1c = th_1c_;
                th_1s = th_1s_;
            }
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

    class FlappingState
    {
        private:
            double beta_0;
            double beta_1c;
            double beta_1s;

        public:
            FlappingState() 
            {
                beta_0 = 0;
                beta_1c = 0;
                beta_1s = 0;
            };
            FlappingState(double beta_0_, double beta_1c_, double beta_1s_)
            {
                beta_0  = beta_0_;
                beta_1c = beta_1c_;
                beta_1s = beta_1s_;
            }
            double _0() const { return beta_0; };
            double _1c() const { return beta_1c; };
            double _1s() const { return beta_1s; };
            double _1cw(double psi_w) const
            {
                return std::cos(psi_w)*beta_1c + std::sin(psi_w)*beta_1s;
            }
            double _1sw(double psi_w) const
            {
                return -std::sin(psi_w)*beta_1c + std::cos(psi_w)*beta_1s;
            }
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


    class InflowState
    {
        private:
            double lambda_0;
            double lambda_1c;
            double lambda_1s;
        
        public:
            InflowState()
            {
                lambda_0 = 0;
                lambda_1c = 0;
                lambda_1s = 0;
            }
            InflowState(double lambda_0_, double lambda_1c_, double lambda_1s_)
            {
                lambda_0 =  lambda_0_;
                lambda_1c = lambda_1c_;
                lambda_1s = lambda_1s_;
            }
            double _0() const { return lambda_0; };
            double _1c() const { return lambda_1c; };
            double _1s() const { return lambda_1s; };
            double _1cw(double psi_w) const
            {
                return std::cos(psi_w)*lambda_1c + std::sin(psi_w)*lambda_1s;
            }
            double _1sw(double psi_w) const
            {
                return -std::sin(psi_w)*lambda_1c + std::cos(psi_w)*lambda_1s;
            }
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

    class BodyState
    {
        public: // input from gazebo sim

            Vector3d u_L; // velocity vector in hub frame
            Vector3d uDot_L;
            Vector3d om_L;
            Vector3d omDot_L;

            Vector3d u_H; // velocity vector in hub frame
            Vector3d uDot_H;
            Vector3d om_H;
            Vector3d omDot_H;

            Pose3d pose_HL_;
            Pose3d pose_WH_;
            Matrix Delta_HW;

            Vector3d u_W; // velocity vector in wind frame
            //Matrix uDot_W; // acceleration vector in wind frame
            Vector3d om_W; // angular velocity vector in wind frame
            Vector3d omDot_W; // angular acceleration vector in wind frame

        public:

            BodyState() {};

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

            Vector3d omBarPrime_W(double Omega) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return omDot_W;
                }
                return omDot_W/std::pow(Omega,2);
            }

            Vector3d omBar_W(double Omega) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return om_W;
                }
                return om_W/Omega;
            }

            Vector3d omBar_H(double Omega) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return om_H;
                }
                return om_H/Omega;
            }

            double psi_w() const
            {
                double u = u_H.X();
                double v = u_H.Y();
                return std::atan2(v,u);
            }

            double psiDot_w()
            {
                double u = u_H.X();
                double v = u_H.Y();
                double uDot = uDot_H.X();
                double vDot = uDot_H.Y();

                return (vDot/u - v*uDot/std::pow(u,2))/(std::pow(v/u,2) + 1);
            }

            double mu(double Omega, double R) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return u_W.X()/R;
                }
                return u_W.X()/(Omega*R);
            }

            double mu_z(double Omega, double R) const
            {
                if (std::abs(Omega) < TOL)
                {
                    return u_W.Z()/R;
                }
                return u_W.Z()/(Omega*R);
            }
    };


    class BladeForceState
    {

        public:
            Vector3d Forces_H;
            Vector3d Torques_H;

            BladeForceState()
            {
                Forces_H = Vector3d::Zero;
                Torques_H = Vector3d::Zero;
            }
        public: // cache
            double C_T;
            double F_1s_outPlane;
            double F_1c_outPlane;
        
        public:

            BladeForceState(Vector3d Forces_H_, Vector3d Torques_H_)
            {
                Forces_H = Forces_H_;
                Torques_H = Torques_H_;
            }
    };

    class RotorShaftState
    {

        private: 
            double Omega_;

        public:
            double Omega() const { return Omega_; };

            void Omega(double val)
            {
                Omega_ = val;
            }

            double OmegaDot;

            

            RotorShaftState()
            {
                Omega_ = 0;
                OmegaDot = 0;
            }

            double OmegaBarPrime() const
            {
                return OmegaDot/(Omega_*Omega_);
            }

    };
}



#endif