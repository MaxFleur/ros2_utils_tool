#pragma once

#include "AdvancedSettings.hpp"

// Store video out of ROS bag creation parameters
class BagToVideoSettings : public AdvancedSettings {
public:
    BagToVideoSettings(Utils::UI::BagToVideoParameters& parameters,
                       const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::BagToVideoParameters& m_parameters;
};
