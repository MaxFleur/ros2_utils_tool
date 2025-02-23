#include "AdvancedSettings.hpp"

AdvancedSettings::AdvancedSettings(Utils::UI::AdvancedParameters& parameters, const QString& groupName) :
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
    settings.endGroup();

    return true;
}
