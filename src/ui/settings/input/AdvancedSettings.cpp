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

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.targetDirectory, "target_dir");
    setSettingsParameter(settings, m_parameters.showAdvancedOptions, "show_advanced");
    settings.endGroup();

    return true;
}


bool
AdvancedSettings::read()
{
    if (!BasicSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.targetDirectory = settings.value("target_dir").isValid() ? settings.value("target_dir").toString() : "";
    m_parameters.showAdvancedOptions = settings.value("show_advanced").isValid() ? settings.value("show_advanced").toBool() : false;
    settings.endGroup();

    return true;
}
