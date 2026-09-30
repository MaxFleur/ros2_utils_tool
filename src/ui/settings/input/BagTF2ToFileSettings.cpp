#include "BagTF2ToFileSettings.hpp"

BagTF2ToFileSettings::BagTF2ToFileSettings(Parameters::BagTF2ToFileParameters& parameters,
                                           const QString&                      groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
BagTF2ToFileSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "keep_timestamps", m_parameters.keepTimestamps);
    writeParameter(m_groupName, "compact_output", m_parameters.compactOutput);

    return true;
}


bool
BagTF2ToFileSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    m_parameters.keepTimestamps = readParameter(m_groupName, "keep_timestamps", true);
    m_parameters.compactOutput = readParameter(m_groupName, "compact_output", true);

    return true;
}
