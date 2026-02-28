#include "PlayBagSettings.hpp"

PlayBagSettings::PlayBagSettings(Parameters::PlayBagParameters& parameters,
                                 const QString&                 groupName) :
    SelectableBagTopicSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PlayBagSettings::write()
{
    if (!SelectableBagTopicSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "rate", m_parameters.rate);
    writeParameter(m_groupName, "loop", m_parameters.loop);

    return true;
}


bool
PlayBagSettings::read()
{
    if (!SelectableBagTopicSettings::read()) {
        return false;
    }

    m_parameters.rate = readParameter(m_groupName, "rate", 1.0);
    m_parameters.loop = readParameter(m_groupName, "loop", true);

    return true;
}
