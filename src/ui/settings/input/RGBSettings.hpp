#pragma once

#include "AdvancedSettings.hpp"

// Store rgb value parameters
class RGBSettings : public AdvancedSettings {
public:
    RGBSettings(Parameters::RGBParameters& parameters,
                const QString&             groupName);
};
