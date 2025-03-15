#pragma once

#include "VideoSettings.hpp"

// Store video out of ROS bag creation parameters
class BagToVideoSettings : public VideoSettings {
public:
    BagToVideoSettings(Parameters::BagToVideoParameters& parameters,
                       const QString&                    groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::BagToVideoParameters& m_parameters;
};
