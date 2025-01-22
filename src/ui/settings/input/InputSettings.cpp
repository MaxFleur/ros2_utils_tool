#include "InputSettings.hpp"

#include "DialogSettings.hpp"

InputSettings::InputSettings(Utils::UI::InputParameters& parameters, const QString& groupName) :
    BasicSettings(groupName), m_parameters(parameters)
{
}


bool
InputSettings::write()
{
    if (const auto areParametersSaved = DialogSettings::areParametersSaved(); !areParametersSaved) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.sourceDirectory, "source_dir");
    setSettingsParameter(settings, m_parameters.topicName, "topic_name");
    settings.endGroup();

    return true;
}


bool
InputSettings::read()
{
    if (const auto areParametersSaved = DialogSettings::areParametersSaved(); !areParametersSaved) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.sourceDirectory = settings.value("source_dir").isValid() ? settings.value("source_dir").toString() : "";
    m_parameters.topicName = settings.value("topic_name").isValid() ? settings.value("topic_name").toString() : "";
    settings.endGroup();

    return true;
}
