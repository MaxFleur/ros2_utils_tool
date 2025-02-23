#pragma once

#include "AdvancedSettings.hpp"

// Store parameters for image sequence out of ROS bag creation
class BagToImagesSettings : public AdvancedSettings {
public:
    BagToImagesSettings(Utils::UI::BagToImagesParameters& parameters,
                        const QString&                    groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::BagToImagesParameters& m_parameters;
};
