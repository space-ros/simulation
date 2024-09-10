#include "AtmosphereModel.hpp"

using namespace simulation;

// Return density since it is uniform with height
double UniformDensityAtmosphereModel::getDensity(double height)
{
    return rho_0;
}