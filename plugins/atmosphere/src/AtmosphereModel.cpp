#include "AtmosphereModel.hpp"

using namespace simulation;

double UniformDensityAtmosphereModel::getDensity(double height)
{
    return rho_0;
}