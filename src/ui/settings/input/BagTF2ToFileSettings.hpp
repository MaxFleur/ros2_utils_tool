#pragma once

#include "AdvancedSettings.hpp"

// Store bag tf2 to file conversion parameters
class BagTF2ToFileSettings : public AdvancedSettings {
public:
    BagTF2ToFileSettings(Parameters::BagTF2ToFileParameters& parameters,
                         const QString&                      groupName);
};
