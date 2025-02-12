#include "PublishSettings.hpp"

PublishSettings::PublishSettings(Utils::UI::PublishParameters& parameters, const QString& groupName) :
    AdvancedInputSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PublishSettings::write()
{
    if (!AdvancedInputSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.exchangeRedBlueValues, "switch_red_blue");
    setSettingsParameter(settings, m_parameters.loop, "loop");
    setSettingsParameter(settings, m_parameters.useHardwareAcceleration, "hw_acc");
    setSettingsParameter(settings, m_parameters.scale, "scale");
    setSettingsParameter(settings, m_parameters.fps, "fps");
    setSettingsParameter(settings, m_parameters.width, "width");
    setSettingsParameter(settings, m_parameters.height, "height");
    settings.endGroup();

    return true;
}


bool
PublishSettings::read()
{
    if (!AdvancedInputSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.exchangeRedBlueValues = settings.value("switch_red_blue").isValid() ? settings.value("switch_red_blue").toBool() : false;
    m_parameters.loop = settings.value("loop").isValid() ? settings.value("loop").toBool() : false;
    m_parameters.useHardwareAcceleration = settings.value("hw_acc").isValid() ? settings.value("hw_acc").toBool() : false;
    m_parameters.scale = settings.value("scale").isValid() ? settings.value("scale").toBool() : false;
    m_parameters.fps = settings.value("fps").isValid() ? settings.value("fps").toInt() : 30;
    m_parameters.width = settings.value("width").isValid() ? settings.value("width").toInt() : 1280;
    m_parameters.height = settings.value("height").isValid() ? settings.value("height").toInt() : 720;
    settings.endGroup();

    return true;
}
