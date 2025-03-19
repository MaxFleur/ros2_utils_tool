#include "CompressBagSettings.hpp"

CompressBagSettings::CompressBagSettings(Parameters::CompressBagParameters& parameters,
                                         const QString&                     groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
CompressBagSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "compress_per_message", m_parameters.compressPerMessage);
    writeParameter(m_groupName, "delete_source", m_parameters.deleteSource);

    return true;
}


bool
CompressBagSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    m_parameters.compressPerMessage = readParameter(m_groupName, "compress_per_message", false);
    m_parameters.deleteSource = readParameter(m_groupName, "delete_source", false);

    return true;
}
