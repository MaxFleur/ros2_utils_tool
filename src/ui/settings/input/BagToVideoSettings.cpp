#include "BagToVideoSettings.hpp"

BagToVideoSettings::BagToVideoSettings(Parameters::BagToVideoParameters& parameters, const QString& groupName) :
    VideoSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
BagToVideoSettings::write()
{
    if (!VideoSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "format", m_parameters.format);
    writeParameter(m_groupName, "bw_images", m_parameters.useBWImages);
    writeParameter(m_groupName, "lossless_images", m_parameters.lossless);

    return true;
}


bool
BagToVideoSettings::read()
{
    if (!VideoSettings::read()) {
        return false;
    }

    m_parameters.format = readParameter(m_groupName, "format", QString("mp4"));
    m_parameters.useBWImages = readParameter(m_groupName, "bw_images", false);
    m_parameters.lossless = readParameter(m_groupName, "lossless_images", false);

    return true;
}
