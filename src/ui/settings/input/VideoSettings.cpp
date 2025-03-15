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

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.fps, "fps");
    settings.endGroup();

    return true;
}


bool
VideoSettings::read()
{
    if (!RGBSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.fps = settings.value("fps").isValid() ? settings.value("fps").toInt() : 30;
    settings.endGroup();

    return true;
}
