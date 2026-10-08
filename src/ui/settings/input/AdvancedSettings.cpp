#include "AdvancedSettings.hpp"

AdvancedSettings::AdvancedSettings(Parameters::AdvancedParameters& parameters, const QString& groupName) :
    BasicSettings(parameters, groupName)
{
    registerParameter("target_dir", parameters.targetDirectory, QString(""));
    registerParameter("topic_name", parameters.topicName, QString(""));
    registerParameter("show_advanced", parameters.showAdvancedOptions, false);

    read();
}
