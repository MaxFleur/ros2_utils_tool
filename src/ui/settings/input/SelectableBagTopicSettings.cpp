#include "SelectableBagTopicSettings.hpp"

SelectableBagTopicSettings::SelectableBagTopicSettings(Parameters::SelectableBagTopicParameters& parameters, const QString& groupName) :
    BasicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
SelectableBagTopicSettings::write()
{
    if (!BasicSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.remove("topics");
    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_parameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        writeParameter(settings, "name", m_parameters.topics.at(i).name);
        writeParameter(settings, "is_selected", m_parameters.topics.at(i).isSelected);
    }
    settings.endArray();
    settings.endGroup();

    return true;
}


bool
SelectableBagTopicSettings::read()
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
        m_parameters.topics.append({ { readParameter(settings, "name", QString("")) },
                                       readParameter(settings, "is_selected", true) });
    }
    settings.endArray();
    settings.endGroup();

    return true;
}
