#include "PublishSettings.hpp"

PublishSettings::PublishSettings(Parameters::PublishParameters& parameters, const QString& groupName) :
    VideoSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PublishSettings::write()
{
    if (!VideoSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "loop", m_parameters.loop);
    writeParameter(m_groupName, "scale", m_parameters.scale);
    writeParameter(m_groupName, "width", m_parameters.width);
    writeParameter(m_groupName, "height", m_parameters.height);

    return true;
}


bool
PublishSettings::read()
{
    if (!VideoSettings::read()) {
        return false;
    }

    m_parameters.loop = readParameter(m_groupName, "loop", false);
    m_parameters.scale = readParameter(m_groupName, "scale", false);
    m_parameters.width = readParameter(m_groupName, "width", 1280);
    m_parameters.height = readParameter(m_groupName, "height", 720);

    return true;
}
