#include "BagToImagesSettings.hpp"

BagToImagesSettings::BagToImagesSettings(Parameters::BagToImagesParameters& parameters, const QString& groupName) :
    RGBSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
BagToImagesSettings::write()
{
    if (!RGBSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "format", m_parameters.format);
    writeParameter(m_groupName, "quality", m_parameters.quality);
    writeParameter(m_groupName, "bw_images", m_parameters.useBWImages);
    writeParameter(m_groupName, "jpg_optimize", m_parameters.jpgOptimize);
    writeParameter(m_groupName, "png_bilevel", m_parameters.pngBilevel);

    return true;
}


bool
BagToImagesSettings::read()
{
    if (!RGBSettings::read()) {
        return false;
    }

    m_parameters.format = readParameter(m_groupName, "format", QString("jpg"));
    m_parameters.quality = readParameter(m_groupName, "quality", 8);
    m_parameters.useBWImages = readParameter(m_groupName, "bw_images", false);
    m_parameters.jpgOptimize = readParameter(m_groupName, "jpg_optimize", false);
    m_parameters.pngBilevel = readParameter(m_groupName, "png_bilevel", false);

    return true;
}
