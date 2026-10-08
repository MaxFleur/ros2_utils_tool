#pragma once

#include "AdvancedSettings.hpp"

// Store pcds to bag conversion parameters
class PCDsToBagSettings : public AdvancedSettings {
public:
    PCDsToBagSettings(Parameters::PCDsToBagParameters& parameters,
                      const QString&                   groupName);
};
