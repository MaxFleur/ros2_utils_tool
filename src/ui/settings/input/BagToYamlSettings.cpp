#include "BagToYamlSettings.hpp"

#include <QDebug>

BagToYamlSettings::BagToYamlSettings(Parameters::BagToYamlParameters& parameters,
                                     const QString&                   groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
BagToYamlSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "write_single_output_file", m_parameters.writeSingleOutputFile);

    return true;
}


bool
BagToYamlSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    m_parameters.writeSingleOutputFile = readParameter(m_groupName, "write_single_output_file", true);

    return true;
}
