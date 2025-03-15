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

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.useCustomFPS, "custom_fps");
    settings.endGroup();

    return true;
}


bool
VideoToBagSettings::read()
{
    if (!VideoSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.useCustomFPS = settings.value("custom_fps").isValid() ? settings.value("custom_fps").toBool() : false;
    settings.endGroup();

    return true;
}
