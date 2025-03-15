#include "RGBSettings.hpp"

RGBSettings::RGBSettings(Parameters::RGBParameters& parameters, const QString& groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
RGBSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.exchangeRedBlueValues, "switch_red_blue");
    settings.endGroup();

    return true;
}


bool
RGBSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.exchangeRedBlueValues = settings.value("switch_red_blue").isValid() ? settings.value("switch_red_blue").toBool() : false;
    settings.endGroup();

    return true;
}
