#include "DialogSettings.hpp"

DialogSettings::DialogSettings(Parameters::DialogParameters& parameters, const QString& groupName) :
    GeneralSettings(groupName), m_parameters(parameters)
{
    read();
}


bool
DialogSettings::write()
{
    writeParameter(m_groupName, "max_threads", m_parameters.maxNumberOfThreads);
    writeParameter(m_groupName, "hw_acc", m_parameters.useHardwareAcceleration);
    writeParameter(m_groupName, "save_parameters", m_parameters.saveParameters);
    writeParameter(m_groupName, "predefined_topic_names", m_parameters.usePredefinedTopicNames);
    writeParameter(m_groupName, "check_ros2_naming_convention", m_parameters.checkROS2NameConform);

    return true;
}


bool
DialogSettings::read()
{
    m_parameters.maxNumberOfThreads = readParameter(m_groupName, "max_threads", std::thread::hardware_concurrency());
    m_parameters.useHardwareAcceleration = readParameter(m_groupName, "hw_acc", false);
    m_parameters.saveParameters = readParameter(m_groupName, "save_parameters", false);
    m_parameters.usePredefinedTopicNames = readParameter(m_groupName, "predefined_topic_names", false);
    m_parameters.checkROS2NameConform = readParameter(m_groupName, "check_ros2_naming_convention", false);

    return true;
}
