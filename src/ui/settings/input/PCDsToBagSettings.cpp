#include "PCDsToBagSettings.hpp"

PCDsToBagSettings::PCDsToBagSettings(Parameters::PCDsToBagParameters& parameters,
                                     const QString&                   groupName) :
    AdvancedSettings(parameters, groupName)
{
    registerParameter("rate", parameters.rate, 5);

    read();
}
