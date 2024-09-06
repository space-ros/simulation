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

#include "BladeForceModel.hpp"
#include <cmath>

using namespace simulation;

BladeForceState BladeForceModel::computeState(const PitchState &pitchState, const InflowState &inflowState, const BodyState &bodyState, const FlappingState &flapState, const RotorShaftState &shaftState, double rho)
{
    // parameters
    double R = rotor->blade.R;
    double a0 = rotor->blade.a0;
    double s = rotor->solidity();
    double th_tw = rotor->blade.th_tw;
    double N_b = rotor->N_b;
    double K_beta = rotor->K_beta;
    double gamma = rotor->gamma(rho);
    double I_R = rotor->I_R;
    double I_beta = rotor->blade.I_beta;

    double Omega = shaftState.Omega();
    double OmegaDot = shaftState.OmegaDot;


    double mu = bodyState.mu(Omega, R);
    double mu_2 = std::pow(mu,2);
    double mu_z = bodyState.mu_z(Omega, R);
    Vector3d omBar_w = bodyState.omBar_W(Omega);
    double pBar_w = omBar_w.X();
    double qBar_w = omBar_w.Y();
    double psi_w = bodyState.psi_w();

    // pitch state
    double th_0 = pitchState._0();
    double th_1cw = pitchState._1cw(psi_w);
    double th_1sw = pitchState._1sw(psi_w);

    // inflow state
    double lmd_0 = inflowState._0();
    double lmd_1cw = inflowState._1cw(psi_w);
    double lmd_1sw = inflowState._1sw(psi_w);

    // flapping state
    double beta_0 = flapState._0();
    double beta_1c = flapState._1c();
    double beta_1s = flapState._1s();
    double beta_1cw = flapState._1cw(psi_w);
    double beta_1sw = flapState._1sw(psi_w);

    double alfa_1sw = pBar_w - lmd_1sw + beta_1cw + th_1sw;
    double alfa_1cw = qBar_w - lmd_1cw - beta_1sw + th_1cw;

    // compute out of rotor-plane forces
    double F_0_outPlane = th_0*(1/3.0 + mu_2/2.0) + mu/2.0*(th_1sw + pBar_w/2.0) + (mu_z - lmd_0)/2.0 + 1/4.0*(1 + mu_2)*th_tw;

    double C_T = a0*s/2.0*F_0_outPlane;
    double delta = rotor->blade.delta_0 + rotor->blade.delta_2*std::pow(C_T,2);

    double F_1s_outPlane = alfa_1sw/3.0 + mu*(th_0 + mu_z - lmd_0 + 2/3.0*th_tw);
    double F_1c_outPlane = alfa_1cw/3.0 - mu*beta_0/2.0;
    double F_2s_outPlane = mu/2.0*(alfa_1cw/2.0 + (th_1cw-beta_1sw)/2.0 - mu*beta_0);
    double F_2c_outPlane = -mu/2.0*(alfa_1sw/2.0 + (th_1sw + beta_1cw)/2.0 + mu*(th_0 + th_tw/2.0));

    // compute in rotor-plane forces
    double F_1s_inPlane = mu_2/2.0*beta_0*beta_1sw + (mu_z - lmd_0 - mu/4.0*beta_1cw)*(alfa_1sw - th_1sw) - mu/4.0*beta_1sw*(alfa_1cw - th_1cw)
                            + th_0*((alfa_1sw - th_1sw)/3.0 + mu*(mu_z - lmd_0) - mu_2/4.0*beta_1cw)
                            + th_tw*((alfa_1sw - th_1sw)/4.0 + mu/2.0*(mu_z - lmd_0 - beta_1cw*mu/4.0))
                            + th_1sw*((mu_z - lmd_0)/2.0 + mu*(3/8.0*(pBar_w - lmd_1sw) + beta_1cw/4.0))
                            + mu/4.0*th_1cw*((qBar_w - lmd_1cw)/2.0 - beta_1sw - mu*beta_0) - delta*mu/a0;

    double F_1c_inPlane = (alfa_1cw - th_1cw - 2*beta_0*mu)*(mu_z - lmd_0 - 3/4.0*mu*beta_1cw) - mu/4.0*beta_1sw*(alfa_1sw - th_1sw)
                            + th_0*((alfa_1cw - th_1cw)/3.0 - mu/2.0*(beta_0 + mu/2.0*beta_1sw))
                            + th_tw*((alfa_1cw - th_1cw)/4.0 - mu*(beta_0/3.0 + beta_1sw*mu/8.0))
                            + th_1cw*((mu_z - lmd_0)/2.0 - mu/4.0*((pBar_w - lmd_1sw)/2.0 - beta_1cw))
                            + mu/4.0*th_1sw*((qBar_w - lmd_1cw)/2.0 - beta_1sw - mu*beta_0);

    double C_xw = a0*s/2.0*((F_0_outPlane/2.0 + F_2c_outPlane/4.0)*beta_1cw + F_1c_outPlane/2.0*beta_0 + F_2s_outPlane/4.0*beta_1sw + F_1s_inPlane/2.0);
    double C_yw = a0*s/2.0*((-F_0_outPlane/2.0 + F_2c_outPlane/4.0)*beta_1sw - F_1s_outPlane/2.0*beta_0 - F_2s_outPlane/4.0*beta_1cw + F_1c_inPlane/2.0);
    double C_zw = - C_T;

    double C_x = std::cos(psi_w)*C_xw - std::sin(psi_w)*C_yw;
    double C_y = std::sin(psi_w)*C_xw + std::cos(psi_w)*C_yw;

    double k = rho*std::pow(Omega*R,2)*M_PI*std::pow(R,2);
    Vector3d Forces_h = Vector3d(k*C_x, k*C_y, k*C_zw); // forces in hub frame

    double L_h = -N_b/2.0*K_beta*beta_1s;
    double M_h = -N_b/2.0*K_beta*beta_1c;
    double C_q = a0*s/2.0*(-(mu_z - lmd_0)*(2*C_T/(a0*s)) + mu*(2*C_xw/(a0*s)) + delta/(4*a0)*(1 + 3*mu_2));
    double Q_R = k*R*C_q;

    double sign = (rotor->ccw) ? 1.0 : -1.0; // sign correction for clockwise rotation (TODO: do proper derivation of formulas for cw rotor)
    L_h += -Q_R/2.0*beta_1c;
    M_h += Q_R/2.0*beta_1s;
    double N_h = 1/2.0*k*R*s*a0*(2*C_q/(a0*s) + 2/gamma*I_R/(N_b*I_beta)*shaftState.OmegaBarPrime());
    Vector3d Torques_h = sign*Vector3d(L_h, M_h, N_h);


    BladeForceState state = BladeForceState(Forces_h, Torques_h);
    state.C_T = C_T;
    state.F_1c_outPlane = F_1c_outPlane;
    state.F_1s_outPlane = F_1s_outPlane;

    
    return state;
}
