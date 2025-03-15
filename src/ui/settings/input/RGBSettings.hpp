#pragma once

#include "AdvancedSettings.hpp"

// Store rgb settings
class RGBSettings : public AdvancedSettings {
public:
    RGBSettings(Parameters::RGBParameters& parameters,
                const QString&             groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Parameters::RGBParameters& m_parameters;
};
