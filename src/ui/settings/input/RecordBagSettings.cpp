#include "RecordBagSettings.hpp"

RecordBagSettings::RecordBagSettings(Parameters::RecordBagParameters& parameters, const QString& groupName) :
    BasicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
RecordBagSettings::write()
{
    if (!BasicSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_parameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        writeParameter(settings, "name", m_parameters.topics.at(i).name);
        writeParameter(settings, "is_selected", m_parameters.topics.at(i).isSelected);
    }
    settings.endArray();
    settings.endGroup();

    writeParameter(m_groupName, "show_advanced", m_parameters.showAdvancedOptions);
    writeParameter(m_groupName, "include_hidden_topics", m_parameters.includeHiddenTopics);
    writeParameter(m_groupName, "include_unpublished_topics", m_parameters.includeUnpublishedTopics);

    return true;
}


bool
RecordBagSettings::read()
{
    if (!BasicSettings::read()) {
        return false;
    }

    QSettings settings;

    settings.beginGroup(m_groupName);
    m_parameters.topics.clear();

    const auto size = settings.beginReadArray("topics");
    for (auto i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        m_parameters.topics.append({ readParameter(settings, "name", QString("")),
                                     readParameter(settings, "is_selected", true) });
    }
    settings.endArray();
    settings.endGroup();

    m_parameters.showAdvancedOptions = readParameter(m_groupName, "show_advanced", false);
    m_parameters.includeHiddenTopics = readParameter(m_groupName, "include_hidden_topics", false);
    m_parameters.includeUnpublishedTopics = readParameter(m_groupName, "include_unpublished_topics", false);

    return true;
}
