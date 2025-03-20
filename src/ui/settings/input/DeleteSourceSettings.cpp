#include "DeleteSourceSettings.hpp"

DeleteSourceSettings::DeleteSourceSettings(Parameters::DeleteSourceParameters& parameters,
                                           const QString&                      groupName) :
    AdvancedSettings(parameters, groupName), m_parameters(parameters)
{
    read();
}


bool
DeleteSourceSettings::write()
{
    if (!AdvancedSettings::write()) {
        return false;
    }

    writeParameter(m_groupName, "delete_source", m_parameters.deleteSource);

    return true;
}


bool
DeleteSourceSettings::read()
{
    if (!AdvancedSettings::read()) {
        return false;
    }

    m_parameters.deleteSource = readParameter(m_groupName, "delete_source", false);

    return true;
}
