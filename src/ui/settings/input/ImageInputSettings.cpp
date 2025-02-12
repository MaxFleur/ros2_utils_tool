#include "ImageInputSettings.hpp"

ImageInputSettings::ImageInputSettings(Utils::UI::ImageInputParameters& parameters, const QString& groupName) :
    AdvancedInputSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
ImageInputSettings::write()
{
    if (!AdvancedInputSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.format, "format");
    setSettingsParameter(settings, m_parameters.quality, "quality");
    setSettingsParameter(settings, m_parameters.exchangeRedBlueValues, "switch_red_blue");
    setSettingsParameter(settings, m_parameters.useBWImages, "bw_images");
    setSettingsParameter(settings, m_parameters.jpgOptimize, "jpg_optimize");
    setSettingsParameter(settings, m_parameters.pngBilevel, "png_bilevel");
    settings.endGroup();

    return true;
}


bool
ImageInputSettings::read()
{
    if (!AdvancedInputSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.format = settings.value("format").isValid() ? settings.value("format").toString() : "jpg";
    m_parameters.quality = settings.value("quality").isValid() ? settings.value("quality").toInt() : 8;
    m_parameters.exchangeRedBlueValues = settings.value("switch_red_blue").isValid() ? settings.value("switch_red_blue").toBool() : false;
    m_parameters.useBWImages = settings.value("bw_images").isValid() ? settings.value("bw_images").toBool() : false;
    m_parameters.jpgOptimize = settings.value("jpg_optimize").isValid() ? settings.value("jpg_optimize").toBool() : false;
    m_parameters.pngBilevel = settings.value("png_bilevel").isValid() ? settings.value("png_bilevel").toBool() : false;
    settings.endGroup();

    return true;
}
