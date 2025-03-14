#include "DialogSettings.hpp"

DialogSettings::DialogSettings(Parameters::DialogParameters& parameters, const QString& groupName) :
    GeneralSettings(groupName), m_parameters(parameters)
{
    read();
}


bool
DialogSettings::write()
{
    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.maxNumberOfThreads, "max_threads");
    setSettingsParameter(settings, m_parameters.useHardwareAcceleration, "hw_acc");
    setSettingsParameter(settings, m_parameters.saveParameters, "save_parameters");
    setSettingsParameter(settings, m_parameters.usePredefinedTopicNames, "predefined_topic_names");
    setSettingsParameter(settings, m_parameters.checkROS2NameConform, "check_ros2_naming_convention");
    settings.endGroup();

    return true;
}


bool
DialogSettings::read()
{
    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.maxNumberOfThreads = settings.value("max_threads").isValid() ? settings.value("max_threads").toInt() : std::thread::hardware_concurrency();
    m_parameters.useHardwareAcceleration = settings.value("hw_acc").isValid() ? settings.value("hw_acc").toBool() : false;
    m_parameters.saveParameters = settings.value("save_parameters").isValid() ? settings.value("save_parameters").toBool() : false;
    m_parameters.usePredefinedTopicNames = settings.value("predefined_topic_names").isValid() ? settings.value("predefined_topic_names").toBool() : true;
    m_parameters.checkROS2NameConform = settings.value("check_ros2_naming_convention").isValid() ? settings.value("check_ros2_naming_convention").toBool() : false;
    settings.endGroup();

    return true;
}
