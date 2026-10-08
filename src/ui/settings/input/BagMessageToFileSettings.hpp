#pragma once

#include "AdvancedSettings.hpp"

// Store bag message to file conversion parameters
class BagMessageToFileSettings : public AdvancedSettings {
public:
    BagMessageToFileSettings(Parameters::BagMessageToFileParameters& parameters,
                             const QString&                          groupName);
};
