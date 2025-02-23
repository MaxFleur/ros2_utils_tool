#pragma once

#include "AdvancedSettings.hpp"

// Store bag creation parameters
class VideoToBagSettings : public AdvancedSettings {
public:
    VideoToBagSettings(Utils::UI::VideoToBagParameters& parameters,
                       const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::VideoToBagParameters& m_parameters;
};
