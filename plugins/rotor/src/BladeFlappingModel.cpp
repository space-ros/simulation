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

#include "BladeFlappingModel.hpp"
#include <cmath>
#include <gz/common/Console.hh>

using namespace simulation;

/// Quasi static computation of flapping state using a center spring model, for further information
/// the reader can refer to Helicopter Flight dynamics by Gareth D. Padfield.
FlappingState CenterSpringModel::computeState(const PitchState &pitchState, const InflowState &inflowState, const BodyState &bodyState, double Omega, double rho)
{
    double th_TW = rotor->blade.th_tw;
    double R = rotor->blade.R;
    double mu = bodyState.mu(Omega, R);
    double mu_z = bodyState.mu_z(Omega, R);
    double psi_w = bodyState.psi_w();

    updateModelMatrices(rho, mu, Omega);

    Matrix th = pitchState2Vector(pitchState, psi_w);
    Matrix lambda = inflowState2Vector(inflowState, psi_w, mu_z);
    Matrix omega = bodyState2Vector(bodyState, Omega);
    auto tmpVec = A_beta_th*th + A_beta_lmd*lambda + A_beta_om*omega;

    return FlappingState(tmpVec.x(), tmpVec.y(), tmpVec.z());
}

void CenterSpringModel::updateModelMatrices(double rho, double mu, double Omega)
{
    // Compute parameters
    BladeConfig blade = rotor->blade;
    double gamma = (rho*blade.c*blade.a0*std::pow(blade.R,4))/blade.I_beta;
    double lambda_beta_2 = 1 + rotor->K_beta/(blade.I_beta*std::pow(Omega,2));

    double S_beta = (lambda_beta_2-1)/(gamma/8.0);
    double eta_beta = -1/(1 + std::pow(S_beta,2));

    double k = gamma/(8*lambda_beta_2);
    double mu_2 = std::pow(mu,2);

    // Compute matrix that relates flapping state to pitch state
    A_beta_th[0][0] = 1 + mu_2; A_beta_th[0][1] = 4/5.0 + 2/3.0*mu_2;
    A_beta_th[0][2] = 4/3.0*mu; A_beta_th[0][3] = 0;
    A_beta_th[1][0] = eta_beta*4/3.0*mu*(S_beta*(1 + mu_2) + 16*lambda_beta_2/gamma*(1 + mu_2/2.0));
    A_beta_th[1][1] = eta_beta*2*mu*(8*lambda_beta_2/gamma*(1 + mu_2/2.0) + 8/15.0*S_beta*(1 + 5/2.0*mu_2));
    A_beta_th[1][2] = eta_beta*(8*lambda_beta_2/gamma*(1 + 2*mu_2) + std::pow(4/3.0*mu,2)*S_beta);
    A_beta_th[1][3] = -eta_beta*S_beta*8*lambda_beta_2/gamma*(1 + mu_2/2.0); // checked until here
    A_beta_th[2][0] = eta_beta*4/3.0*mu*(1 + mu_2/2.0 - 2*S_beta*8*lambda_beta_2/gamma);
    A_beta_th[2][1] = eta_beta*2*mu*(8/15.0*(1 + mu_2/3.0) - S_beta*8*lambda_beta_2/gamma);
    A_beta_th[2][2] = eta_beta*(16/9.0*mu_2 - S_beta*8*lambda_beta_2/gamma*(1 + 3/2.0*mu_2));
    A_beta_th[2][3] = -eta_beta*8*lambda_beta_2/gamma*(1 - std::pow(mu,4)/2.0);
    A_beta_th = A_beta_th*(gamma/(8*lambda_beta_2));

    // Compute matrix that relates flapping state to inflow state
    A_beta_lmd[0][0] = 4/3.0; A_beta_lmd[0][1] = -2/3.0*mu; A_beta_lmd[0][2] = 0;
    A_beta_lmd[1][0] = eta_beta*mu*(16/9.0*S_beta + 16*lambda_beta_2/gamma*(1 + mu_2/2.0));
    A_beta_lmd[1][1] = -eta_beta*(8*lambda_beta_2/gamma*(1 + mu_2/2.0) + S_beta*8/9.0*mu_2);
    A_beta_lmd[1][2] = eta_beta*8*lambda_beta_2/gamma*S_beta;
    A_beta_lmd[2][0] = eta_beta*mu*(16/9.0*(1 - mu_2/2.0) - S_beta*16*lambda_beta_2/gamma);
    A_beta_lmd[2][1] = eta_beta*(8*lambda_beta_2/gamma*S_beta - 8/9.0*mu_2);
    A_beta_lmd[2][2] = eta_beta*8*lambda_beta_2/gamma*(1 - mu_2/2.0);
    A_beta_lmd = A_beta_lmd*(gamma/(8*lambda_beta_2));

    // Compute matrix that relates flapping state to body state
    A_beta_om[0][0] = 0; A_beta_om[0][1] = 0; A_beta_om[0][2] = 2/3.0*mu; A_beta_om[0][3] = 0;
    A_beta_om[1][0] = eta_beta*std::pow(8/gamma,2)*lambda_beta_2*(1 + mu_2/2.0);
    A_beta_om[1][1] = -eta_beta*std::pow(8/gamma,2)*lambda_beta_2*S_beta;
    A_beta_om[1][2] = eta_beta*(8*lambda_beta_2/gamma*(1 + mu_2/2.0 - 16*S_beta/gamma) + S_beta*8/9.0*mu_2);
    A_beta_om[1][3] = -eta_beta*8*lambda_beta_2/gamma*(S_beta + 16/gamma*(1 + mu_2/2.0));
    A_beta_om[2][0] = -eta_beta*S_beta*64*lambda_beta_2/std::pow(gamma,2);
    A_beta_om[2][1] = eta_beta*64*lambda_beta_2/std::pow(gamma,2)*(mu_2/2.0-1);
    A_beta_om[2][2] = (eta_beta*8*lambda_beta_2/gamma*(16/gamma*(mu_2/2.0-1)-S_beta)+8/9.0*mu_2);
    A_beta_om[2][3] = eta_beta*8*lambda_beta_2/gamma*(16*S_beta/gamma + mu_2/2.0 - 1);
    A_beta_om = A_beta_om*(gamma/(8*lambda_beta_2));
}

Matrix CenterSpringModel::inflowState2Vector(const InflowState &inflowState, double psi_w, double mu_z)
{
    double lmd_1cw = inflowState._1cw(psi_w);
    double lmd_1sw = inflowState._1sw(psi_w);
    Matrix vector = {{mu_z - inflowState._0()}, {lmd_1sw}, {lmd_1cw}};
    return vector;
}

Matrix CenterSpringModel::pitchState2Vector(const PitchState &pitchState, double psi_w)
{
    double th_tw = rotor->blade.th_tw;
    double th_1cw = pitchState._1cw(psi_w);
    double th_1sw = pitchState._1sw(psi_w);
    Matrix vector = {{pitchState._0()},{th_tw},{th_1sw},{th_1cw}};
    return vector;
}

Matrix CenterSpringModel::bodyState2Vector(const BodyState &bodyState, double Omega)
{
    Vector3d omBarPrime_W = bodyState.omBarPrime_W(Omega);
    Vector3d omBar_W = bodyState.omBar_W(Omega);
    std::vector<std::vector<double>> columnMatrix = {
        {omBarPrime_W.X()}, 
        {omBarPrime_W.Y()}, 
        {omBar_W.X()}, 
        {omBar_W.Y()}, 
    };
    return Matrix(columnMatrix);
}

/// Simplified model of flapping dynamics, the rotor acts almost as a gyroscope, for further information
/// the reader can refer to Principles of Helicopter Aerodynamics by J. Gordon Leishman.
FlappingState GordonFlappingModel::computeState(const PitchState &pitchState, const InflowState &inflowState, const BodyState &bodyState, double Omega, double rho)
{
    double th_TW = rotor->blade.th_tw;
    double R = rotor->blade.R;
    double mu = bodyState.mu(Omega, R);

    double sign = (rotor->ccw) ? 1.0 : -1.0; // TODO: a proper derivation for the cw case is required

    double th_0 = pitchState._0();
    double th_1c = pitchState._1c();
    double th_1s = pitchState._1s();

    double lmd = inflowState._0();

    BladeConfig blade = rotor->blade;
    double gamma = (rho*blade.c*blade.a0*std::pow(blade.R,4))/blade.I_beta;
    double beta_0 = gamma*(th_0/8.0*(1 + std::pow(mu,2)) + th_TW/10.0*(1 + 5/6.0*std::pow(mu,2)) + mu/6.0*th_1s - lmd/6.0);
    double beta_1s = sign*th_1c + (-4/3.0*mu*beta_0)/(1 + 1/2.0*std::pow(mu,2));
    double beta_1c = -sign*th_1s + (-8/3.0*mu*(th_0 - 3/4.0*lmd + 3/4.0*mu*th_1s + 3/4.0*th_TW))/(1 - 1/2.0*std::pow(mu,2));


    return FlappingState(beta_0, beta_1c, beta_1s);
}
