#include "BasicSettings.hpp"

#include "DialogSettings.hpp"

BasicSettings::BasicSettings(Parameters::BasicParameters& parameters, const QString& groupName) :
    GeneralSettings(groupName), m_parameters(parameters)
{
}


bool
BasicSettings::write()
{
    if (!DialogSettings::getStaticParameter("save_parameters", false)) {
        return false;
    }

    writeParameter(m_groupName, "source_dir", m_parameters.sourceDirectory);
    writeParameter(m_groupName, "topic_name", m_parameters.topicName);

    return true;
}


bool
BasicSettings::read()
{
    if (!DialogSettings::getStaticParameter("save_parameters", false)) {
        return false;
    }

    m_parameters.sourceDirectory = readParameter(m_groupName, "source_dir", QString(""));
    m_parameters.topicName = readParameter(m_groupName, "topic_name", QString(""));

    return true;
}
