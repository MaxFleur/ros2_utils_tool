#include "PublishSettings.hpp"

PublishSettings::PublishSettings(Parameters::PublishParameters& parameters, const QString& groupName) :
    VideoSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PublishSettings::write()
{
    if (!VideoSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.loop, "loop");
    setSettingsParameter(settings, m_parameters.scale, "scale");
    setSettingsParameter(settings, m_parameters.width, "width");
    setSettingsParameter(settings, m_parameters.height, "height");
    settings.endGroup();

    return true;
}


bool
PublishSettings::read()
{
    if (!VideoSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.loop = settings.value("loop").isValid() ? settings.value("loop").toBool() : false;
    m_parameters.scale = settings.value("scale").isValid() ? settings.value("scale").toBool() : false;
    m_parameters.width = settings.value("width").isValid() ? settings.value("width").toInt() : 1280;
    m_parameters.height = settings.value("height").isValid() ? settings.value("height").toInt() : 720;
    settings.endGroup();

    return true;
}
