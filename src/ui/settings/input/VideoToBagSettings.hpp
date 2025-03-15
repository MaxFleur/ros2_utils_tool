#pragma once

#include "VideoSettings.hpp"

// Store bag creation parameters
class VideoToBagSettings : public VideoSettings {
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
