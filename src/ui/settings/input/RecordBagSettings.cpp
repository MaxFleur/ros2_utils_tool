#include "RecordBagSettings.hpp"

RecordBagSettings::RecordBagSettings(Parameters::RecordBagParameters& parameters, const QString& groupName) :
    SelectableBagTopicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
RecordBagSettings::write()
{
    if (!SelectableBagTopicSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "show_advanced", m_parameters.showAdvancedOptions);
    writeParameter(m_groupName, "include_hidden_topics", m_parameters.includeHiddenTopics);
    writeParameter(m_groupName, "include_unpublished_topics", m_parameters.includeUnpublishedTopics);

    return true;
}


bool
RecordBagSettings::read()
{
    if (!SelectableBagTopicSettings::read()) {
        return false;
    }

    m_parameters.showAdvancedOptions = readParameter(m_groupName, "show_advanced", false);
    m_parameters.includeHiddenTopics = readParameter(m_groupName, "include_hidden_topics", false);
    m_parameters.includeUnpublishedTopics = readParameter(m_groupName, "include_unpublished_topics", false);

    return true;
}
