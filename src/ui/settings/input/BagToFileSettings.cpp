#include "BagToFileSettings.hpp"

BagToFileSettings::BagToFileSettings(Parameters::BagToFileParameters& parameters,
                                     const QString&                   groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
BagToFileSettings::write()
{
    if (!AdvancedSettings::write()) {
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

    writeParameter(m_groupName, "format", m_parameters.format);
    writeParameter(m_groupName, "all_topics", m_parameters.allTopics);
    writeParameter(m_groupName, "single_file", m_parameters.singleFile);

    return true;
}


bool
BagToFileSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.topics.clear();

    const auto size = settings.beginReadArray("topics");
    for (auto i = 0; i < size; ++i) {
        settings.setArrayIndex(i);
        m_parameters.topics.append({ readParameter(settings, "name", QString("")), readParameter(settings, "is_selected", false) });
    }
    settings.endArray();
    settings.endGroup();

    m_parameters.format = readParameter(m_groupName, "format", QString("json"));
    m_parameters.allTopics = readParameter(m_groupName, "all_topics", true);
    m_parameters.singleFile = readParameter(m_groupName, "single_file", true);

    return true;
}
