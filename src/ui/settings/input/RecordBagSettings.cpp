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

    writeParameter(m_groupName, "size", m_parameters.maxSizeInMB);
    writeParameter(m_groupName, "duration", m_parameters.maxDurationInSeconds);
    writeParameter(m_groupName, "show_advanced", m_parameters.showAdvancedOptions);
    writeParameter(m_groupName, "include_hidden_topics", m_parameters.includeHiddenTopics);
    writeParameter(m_groupName, "include_unpublished_topics", m_parameters.includeUnpublishedTopics);
    writeParameter(m_groupName, "use_custom_size", m_parameters.useCustomSize);
    writeParameter(m_groupName, "use_custom_duration", m_parameters.useCustomDuration);
    writeParameter(m_groupName, "use_compression", m_parameters.useCompression);
    writeParameter(m_groupName, "is_compression_file", m_parameters.isCompressionFile);

    return true;
}


bool
RecordBagSettings::read()
{
    if (!SelectableBagTopicSettings::read()) {
        return false;
    }

    m_parameters.maxSizeInMB = readParameter(m_groupName, "size", 1024);
    m_parameters.maxDurationInSeconds = readParameter(m_groupName, "duration", 60);
    m_parameters.showAdvancedOptions = readParameter(m_groupName, "show_advanced", false);
    m_parameters.includeHiddenTopics = readParameter(m_groupName, "include_hidden_topics", false);
    m_parameters.includeUnpublishedTopics = readParameter(m_groupName, "include_unpublished_topics", false);
    m_parameters.useCustomSize = readParameter(m_groupName, "use_custom_size", false);
    m_parameters.useCustomDuration = readParameter(m_groupName, "use_custom_duration", false);
    m_parameters.useCompression = readParameter(m_groupName, "use_compression", false);
    m_parameters.isCompressionFile = readParameter(m_groupName, "is_compression_file", false);

    return true;
}
