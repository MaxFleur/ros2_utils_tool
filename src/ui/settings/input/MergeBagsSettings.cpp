#include "MergeBagsSettings.hpp"

MergeBagsSettings::MergeBagsSettings(Parameters::MergeBagsParameters& parameters,
                                     const QString&                   groupName) :
    DeleteSourceSettings(parameters, groupName)
{
    registerArrayParameter("topics", parameters.topics,
                           [] (QSettings& settings, Parameters::MergeBagsParameters::MergeBagTopic& topic) {
        topic.name = readParameter(settings, "name", QString(""));
        topic.isSelected = readParameter(settings, "is_selected", false);
        topic.bagDir = readParameter(settings, "dir", QString(""));
    },
                           [] (QSettings& settings, const Parameters::MergeBagsParameters::MergeBagTopic& topic) {
        writeParameter(settings, "name", topic.name);
        writeParameter(settings, "is_selected", topic.isSelected);
        writeParameter(settings, "dir", topic.bagDir);
    });
    registerParameter("second_source", parameters.secondSourceDirectory, QString(""));
    registerParameter("compress_target", parameters.compressTarget, false);

    read();
}
