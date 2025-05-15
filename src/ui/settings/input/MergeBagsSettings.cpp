#include "MergeBagsSettings.hpp"

MergeBagsSettings::MergeBagsSettings(Parameters::MergeBagsParameters& parameters,
                                     const QString&                   groupName) :
    DeleteSourceSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
MergeBagsSettings::write()
{
    if (!DeleteSourceSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);

    settings.beginWriteArray("topics");
    for (auto i = 0; i < m_parameters.topics.size(); ++i) {
        settings.setArrayIndex(i);
        writeParameter(settings, "name", m_parameters.topics.at(i).name);
        writeParameter(settings, "dir", m_parameters.topics.at(i).bagDir);
        writeParameter(settings, "is_selected", m_parameters.topics.at(i).isSelected);
    }
    settings.endArray();
    settings.endGroup();

    writeParameter(m_groupName, "second_source", m_parameters.secondSourceDirectory);

    return true;
}


bool
MergeBagsSettings::read()
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
        m_parameters.topics.append({ readParameter(settings, "name", QString("")), readParameter(settings, "dir", QString("")),
                                     readParameter(settings, "is_selected", false) });
    }
    settings.endArray();
    settings.endGroup();

    m_parameters.secondSourceDirectory = readParameter(m_groupName, "second_source", QString(""));

    return true;
}
