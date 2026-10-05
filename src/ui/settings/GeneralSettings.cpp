#include "GeneralSettings.hpp"

#include "DialogSettings.hpp"

bool
GeneralSettings::mainReadWriteOperation(bool write)
{
    // Will always be true for dialog settings
    if (m_checkSaveGate && !DialogSettings::getStaticParameter("save_parameters", false)) {
        return false;
    }

    QSettings settings;
    settings.beginGroup(m_groupName);
    for (const auto& parameter : m_parameters) {
        write ?  parameter->write(settings) : parameter->read(settings);
    }
    settings.endGroup();

    return true;
}
