#pragma once

#include "AdvancedSettings.hpp"

// Store publishing parameters
class PublishSettings : public AdvancedSettings {
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
