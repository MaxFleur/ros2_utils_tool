#pragma once

#include "AdvancedSettings.hpp"

// Store video out of ROS bag creation parameters
class BagToVideoSettings : public AdvancedSettings {
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
