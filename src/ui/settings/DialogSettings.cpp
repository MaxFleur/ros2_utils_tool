#include "DialogSettings.hpp"

DialogSettings::DialogSettings(Parameters::DialogParameters& parameters, const QString& groupName) :
    GeneralSettings(groupName), m_parameters(parameters)
{
    read();
}


bool
DialogSettings::areParametersSaved()
{
    QSettings settings;
    settings.beginGroup("dialog");
    const auto savedValue = settings.value("save_parameters").isValid() ? settings.value("save_parameters").toBool() : false;
    settings.endGroup();

    return savedValue;
}


int
DialogSettings::maximumNumberOfThreads()
{
    QSettings settings;
    settings.beginGroup("dialog");
    const auto savedValue = settings.value("max_threads").isValid() ? settings.value("max_threads").toInt() : std::thread::hardware_concurrency();
    settings.endGroup();

    return savedValue;
}


bool
DialogSettings::write()
{
    QSettings settings;
    settings.beginGroup(m_groupName);
    settings.setValue("max_threads", m_parameters.maxNumberOfThreads);
    settings.setValue("save_parameters", m_parameters.saveParameters);
    settings.setValue("predefined_topic_names", m_parameters.usePredefinedTopicNames);
    settings.setValue("check_ros2_naming_convention", m_parameters.checkROS2NameConform);
    settings.endGroup();

    return true;
}


bool
DialogSettings::read()
{
    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.maxNumberOfThreads = settings.value("max_threads").isValid() ? settings.value("max_threads").toInt() : std::thread::hardware_concurrency();
    m_parameters.saveParameters = settings.value("save_parameters").isValid() ? settings.value("save_parameters").toBool() : false;
    m_parameters.usePredefinedTopicNames = settings.value("predefined_topic_names").isValid() ? settings.value("predefined_topic_names").toBool() : true;
    m_parameters.checkROS2NameConform = settings.value("check_ros2_naming_convention").isValid() ?
                                        settings.value("check_ros2_naming_convention").toBool() :
                                        false;
    settings.endGroup();

    return true;
}
