#include "EditBagSettings.hpp"

EditBagSettings::EditBagSettings(Parameters::EditBagParameters& parameters,
                                 const QString&                 groupName) :
    DeleteSourceSettings(parameters, groupName)
{
    const auto readParameters = [] (QSettings& settings, Parameters::EditBagParameters::EditBagTopic& topic) {
        topic.name = readParameter(settings, "name", QString(""));
        topic.isSelected = readParameter(settings, "is_selected", false);
        topic.renamedName = readParameter(settings, "renamed_name", QString(""));
        topic.lowerBoundary = readParameter(settings, "lower_boundary", static_cast<size_t>(0));
        topic.upperBoundary = readParameter(settings, "upper_boundary", static_cast<size_t>(0));
    };
    const auto writeParameters = [] (QSettings& settings, const Parameters::EditBagParameters::EditBagTopic& topic) {
        writeParameter(settings, "name", topic.name);
        writeParameter(settings, "is_selected", topic.isSelected);
        writeParameter(settings, "renamed_name", topic.renamedName);
        writeParameter(settings, "lower_boundary", topic.lowerBoundary);
        writeParameter(settings, "upper_boundary", topic.upperBoundary);
    };

    registerArrayParameter("topics", parameters.topics, readParameters, writeParameters);
    registerParameter("update_timestamps", parameters.updateTimestamps, false);
    registerParameter("compress_target", parameters.compressTarget, false);

    read();
}
