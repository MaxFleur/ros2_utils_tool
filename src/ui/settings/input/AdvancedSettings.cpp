#include "AdvancedSettings.hpp"

AdvancedSettings::AdvancedSettings(Parameters::AdvancedParameters& parameters, const QString& groupName) :
    BasicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
AdvancedSettings::write()
{
    if (!BasicSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "target_dir", m_parameters.targetDirectory);
    writeParameter(m_groupName, "show_advanced", m_parameters.showAdvancedOptions);

    return true;
}


bool
AdvancedSettings::read()
{
    if (!BasicSettings::read()) {
        return false;
    }

    m_parameters.targetDirectory = readParameter(m_groupName, "target_dir", QString(""));
    m_parameters.showAdvancedOptions = readParameter(m_groupName, "show_advanced", false);

    return true;
}
