#include "PlayBagSettings.hpp"

PlayBagSettings::PlayBagSettings(Parameters::PlayBagParameters& parameters,
                                 const QString&                 groupName) :
    BasicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PlayBagSettings::write()
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

    writeParameter(m_groupName, "rate", m_parameters.rate);
    writeParameter(m_groupName, "loop", m_parameters.loop);

    return true;
}


bool
PlayBagSettings::read()
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
                                       readParameter(settings, "is_selected", false) });
    }
    settings.endArray();
    settings.endGroup();

    m_parameters.rate = readParameter(m_groupName, "rate", 1.0);
    m_parameters.loop = readParameter(m_groupName, "loop", true);

    return true;
}
