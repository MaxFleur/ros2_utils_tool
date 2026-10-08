#pragma once

#include "RGBSettings.hpp"

// Store bag to images conversion parameters
class BagToImagesSettings : public RGBSettings {
public:
    BagToImagesSettings(Parameters::BagToImagesParameters& parameters,
                        const QString&                     groupName);
};
