#pragma once

#include "RGBSettings.hpp"

// Store video settings
class VideoSettings : public RGBSettings {
public:
    VideoSettings(Parameters::VideoParameters& parameters,
                  const QString&               groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Parameters::VideoParameters& m_parameters;
};
