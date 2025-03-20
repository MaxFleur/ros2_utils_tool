#include "CompressBagSettings.hpp"

CompressBagSettings::CompressBagSettings(Parameters::CompressBagParameters& parameters,
                                         const QString&                     groupName) :
    DeleteSourceSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
CompressBagSettings::write()
{
    if (!DeleteSourceSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "compress_per_message", m_parameters.compressPerMessage);

    return true;
}


bool
CompressBagSettings::read()
{
    if (!DeleteSourceSettings::read()) {
        return false;
    }

    m_parameters.compressPerMessage = readParameter(m_groupName, "compress_per_message", false);

    return true;
}
