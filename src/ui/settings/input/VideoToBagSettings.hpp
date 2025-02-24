#pragma once

#include "AdvancedSettings.hpp"

// Store bag creation parameters
class VideoToBagSettings : public AdvancedSettings {
public:
    VideoToBagSettings(Parameters::VideoToBagParameters& parameters,
                       const QString&                    groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::VideoToBagParameters& m_parameters;
};
