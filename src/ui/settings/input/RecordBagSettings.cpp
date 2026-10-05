#include "RecordBagSettings.hpp"

RecordBagSettings::RecordBagSettings(Parameters::RecordBagParameters& parameters, const QString& groupName) :
    SelectableBagContentSettings(parameters, groupName)
{
    registerParameter("size", parameters.maxSizeInMB, 1024);
    registerParameter("duration", parameters.maxDurationInSeconds, 60);
    registerParameter("include_ros_topics", parameters.includeROSTopics, false);
    registerParameter("show_advanced", parameters.showAdvancedOptions, false);
    registerParameter("include_hidden_topics", parameters.includeHiddenTopics, false);
    registerParameter("include_unpublished_topics", parameters.includeUnpublishedTopics, false);
    registerParameter("use_custom_size", parameters.useCustomSize, false);
    registerParameter("use_custom_duration", parameters.useCustomDuration, false);
    registerParameter("use_compression", parameters.useCompression, false);
    registerParameter("is_compression_file", parameters.isCompressionFile, false);

    read();
}
