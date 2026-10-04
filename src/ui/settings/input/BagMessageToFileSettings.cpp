#include "BagMessageToFileSettings.hpp"

BagMessageToFileSettings::BagMessageToFileSettings(Parameters::BagMessageToFileParameters& parameters,
                                                   const QString&                          groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
BagMessageToFileSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "write_single_output_file", m_parameters.writeSingleOutputFile);
    writeParameter(m_groupName, "is_yaml", m_parameters.isYaml);

    return true;
}


bool
BagMessageToFileSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    m_parameters.writeSingleOutputFile = readParameter(m_groupName, "write_single_output_file", true);
    m_parameters.isYaml = readParameter(m_groupName, "is_yaml", true);

    return true;
}
