#include "VideoSettings.hpp"

VideoSettings::VideoSettings(Parameters::VideoParameters& parameters, const QString& groupName) :
    RGBSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
VideoSettings::write()
{
    if (!RGBSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "fps", m_parameters.fps);

    return true;
}


bool
VideoSettings::read()
{
    if (!RGBSettings::read()) {
        return false;
    }

    m_parameters.fps = readParameter(m_groupName, "fps", 30);

    return true;
}
