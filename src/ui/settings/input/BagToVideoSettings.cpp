#include "BagToVideoSettings.hpp"

BagToVideoSettings::BagToVideoSettings(Parameters::BagToVideoParameters& parameters, const QString& groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
BagToVideoSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.format, "format");
    setSettingsParameter(settings, m_parameters.fps, "fps");
    setSettingsParameter(settings, m_parameters.exchangeRedBlueValues, "switch_red_blue");
    setSettingsParameter(settings, m_parameters.useBWImages, "bw_images");
    setSettingsParameter(settings, m_parameters.lossless, "lossless_images");
    settings.endGroup();

    return true;
}


bool
BagToVideoSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.format = settings.value("format").isValid() ? settings.value("format").toString() : "mp4";
    m_parameters.fps = settings.value("fps").isValid() ? settings.value("fps").toInt() : 30;
    m_parameters.exchangeRedBlueValues = settings.value("switch_red_blue").isValid() ? settings.value("switch_red_blue").toBool() : false;
    m_parameters.useBWImages = settings.value("bw_images").isValid() ? settings.value("bw_images").toBool() : false;
    m_parameters.lossless = settings.value("lossless_images").isValid() ? settings.value("lossless_images").toBool() : false;
    settings.endGroup();

    return true;
}
