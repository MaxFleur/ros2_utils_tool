#include "DummyBagSettings.hpp"

DummyBagSettings::DummyBagSettings(Parameters::DummyBagParameters& parameters, const QString& groupName) :
    BasicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
DummyBagSettings::write()
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
        writeParameter(settings, "type", m_parameters.topics.at(i).type);
    }
    settings.endArray();
    settings.endGroup();

    writeParameter(m_groupName, "msg_count", m_parameters.messageCount);
    writeParameter(m_groupName, "rate", m_parameters.rate);
    writeParameter(m_groupName, "use_custom_rate", m_parameters.useCustomRate);

    return true;
}


bool
DummyBagSettings::read()
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
        m_parameters.topics.append({ { readParameter(settings, "name", QString("")) }, readParameter(settings, "type", QString("")) });
    }
    settings.endArray();
    settings.endGroup();

    m_parameters.messageCount = readParameter(m_groupName, "msg_count", 100);
    m_parameters.rate = readParameter(m_groupName, "rate", 10);
    m_parameters.useCustomRate = readParameter(m_groupName, "use_custom_rate", false);

    return true;
}
