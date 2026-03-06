#include "RGBSettings.hpp"

RGBSettings::RGBSettings(Parameters::RGBParameters& parameters, const QString& groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
RGBSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "switch_red_blue", m_parameters.exchangeRedBlueValues);
    writeParameter(m_groupName, "use_compression", m_parameters.useCompression);

    return true;
}


bool
RGBSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    m_parameters.exchangeRedBlueValues = readParameter(m_groupName, "switch_red_blue", false);
    m_parameters.useCompression = readParameter(m_groupName, "use_compression", false);

    return true;
}
