#pragma once

#include "VideoSettings.hpp"

// Store publish parameters
class PublishSettings : public VideoSettings {
public:
    PublishSettings(Parameters::PublishParameters& parameters,
                    const QString&                 groupName);
};
