#pragma once

#include "VideoSettings.hpp"

// Store the specific parameters for the video to bag tool
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
