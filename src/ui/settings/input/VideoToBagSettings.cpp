#include "VideoToBagSettings.hpp"

VideoToBagSettings::VideoToBagSettings(Parameters::VideoToBagParameters& parameters,
                                       const QString&                    groupName) :
    VideoSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
VideoToBagSettings::write()
{
    if (!VideoSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "custom_fps", m_parameters.useCustomFPS);

    return true;
}


bool
VideoToBagSettings::read()
{
    if (!VideoSettings::read()) {
        return false;
    }

    m_parameters.useCustomFPS = readParameter(m_groupName, "custom_fps", false);

    return true;
}
