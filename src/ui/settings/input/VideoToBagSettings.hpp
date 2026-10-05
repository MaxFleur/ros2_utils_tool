#pragma once

#include "VideoSettings.hpp"

// Store video to bag conversion parameters
class VideoToBagSettings : public VideoSettings {
public:
    VideoToBagSettings(Parameters::VideoToBagParameters& parameters,
                       const QString&                    groupName);
};
