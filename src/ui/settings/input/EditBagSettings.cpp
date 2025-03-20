#include "EditBagSettings.hpp"

EditBagSettings::EditBagSettings(Parameters::EditBagParameters& parameters,
                                 const QString&                 groupName) :
    DeleteSourceSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
EditBagSettings::write()
{
    if (!DeleteSourceSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_parameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        writeParameter(settings, "renamed_name", m_parameters.topics.at(i).renamedTopicName);
        writeParameter(settings, "original_name", m_parameters.topics.at(i).originalTopicName);
        writeParameter(settings, "is_selected", m_parameters.topics.at(i).isSelected);
        writeParameter(settings, "lower_boundary", m_parameters.topics.at(i).lowerBoundary);
        writeParameter(settings, "upper_boundary", m_parameters.topics.at(i).upperBoundary);
    }
    settings.endArray();
    settings.endGroup();

    writeParameter(m_groupName, "update_timestamps", m_parameters.updateTimestamps);

    return true;
}


bool
EditBagSettings::read()
{
    if (!DeleteSourceSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.topics.clear();

    const auto size = settings.beginReadArray("topics");
    for (auto i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        m_parameters.topics.append({ readParameter(settings, "renamed_name", QString("")), readParameter(settings, "original_name", QString("")),
                                     static_cast<size_t>(readParameter(settings, "lower_boundary", 0)),
                                     static_cast<size_t>(readParameter(settings, "upper_boundary", 0)),
                                     readParameter(settings, "is_selected", false) });
    }
    settings.endArray();
    settings.endGroup();

    m_parameters.updateTimestamps = readParameter(m_groupName, "update_timestamps", false);

    return true;
}
