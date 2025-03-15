#pragma once

#include "VideoSettings.hpp"

// Store publishing parameters
class PublishSettings : public VideoSettings {
public:
    PublishSettings(Parameters::PublishParameters& parameters,
                    const QString&                 groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::PublishParameters& m_parameters;
};
