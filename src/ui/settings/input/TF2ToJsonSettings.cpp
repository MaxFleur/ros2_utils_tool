#include "TF2ToJsonSettings.hpp"

TF2ToJsonSettings::TF2ToJsonSettings(Parameters::TF2ToJsonParameters& parameters,
                                     const QString&                   groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
TF2ToJsonSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "compact_output", m_parameters.compactOutput);
    writeParameter(m_groupName, "keep_timestamps", m_parameters.keepTimestamps);

    return true;
}


bool
TF2ToJsonSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    m_parameters.compactOutput = readParameter(m_groupName, "compact_output", true);
    m_parameters.keepTimestamps = readParameter(m_groupName, "keep_timestamps", true);

    return true;
}
