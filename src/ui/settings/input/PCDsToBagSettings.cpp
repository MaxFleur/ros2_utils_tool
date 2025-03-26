#include "PCDsToBagSettings.hpp"

PCDsToBagSettings::PCDsToBagSettings(Parameters::PCDsToBagParameters& parameters,
                                     const QString&                   groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PCDsToBagSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "rate", m_parameters.rate);

    return true;
}


bool
PCDsToBagSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    m_parameters.rate = readParameter(m_groupName, "rate", 5);

    return true;
}
