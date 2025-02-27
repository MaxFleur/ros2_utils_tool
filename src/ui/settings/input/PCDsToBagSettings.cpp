#include "PCDsToBagSettings.hpp"

PCDsToBagSettings::PCDsToBagSettings(Parameters::PCDsToBagParameters& parameters,
                                     const QString&                   groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
PCDsToBagSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    setSettingsParameter(settings, m_parameters.rate, "rate");
    settings.endGroup();

    return true;
}


bool
PCDsToBagSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    m_parameters.rate = settings.value("rate").isValid() ? settings.value("rate").toInt() : 5;
    settings.endGroup();

    return true;
}
