#pragma once

#include "VideoSettings.hpp"

// Store bag to video tool parameters
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
