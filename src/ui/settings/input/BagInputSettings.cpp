#include "BagInputSettings.hpp"

BagInputSettings::BagInputSettings(Utils::UI::BagInputParameters& parameters,
                                   const QString&                 groupName) :
    AdvancedInputSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
BagInputSettings::write()
{
    if (!AdvancedInputSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.fps, "fps");
    setSettingsParameter(settings, m_parameters.useCustomFPS, "custom_fps");
    setSettingsParameter(settings, m_parameters.useHardwareAcceleration, "hw_acc");
    setSettingsParameter(settings, m_parameters.exchangeRedBlueValues, "switch_red_blue");
    settings.endGroup();

    return true;
}


bool
BagInputSettings::read()
{
    if (!AdvancedInputSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.fps = settings.value("fps").isValid() ? settings.value("fps").toInt() : 30;
    m_parameters.useCustomFPS = settings.value("custom_fps").isValid() ? settings.value("custom_fps").toBool() : false;
    m_parameters.useHardwareAcceleration = settings.value("hw_acc").isValid() ? settings.value("hw_acc").toBool() : false;
    m_parameters.exchangeRedBlueValues = settings.value("switch_red_blue").isValid() ? settings.value("switch_red_blue").toBool() : false;
    settings.endGroup();

    return true;
}
