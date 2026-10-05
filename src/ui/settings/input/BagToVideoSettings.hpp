#pragma once

#include "VideoSettings.hpp"

// Store bag to video conversion parameters
class BagToVideoSettings : public VideoSettings {
public:
    BagToVideoSettings(Parameters::BagToVideoParameters& parameters,
                       const QString&                    groupName);
};
