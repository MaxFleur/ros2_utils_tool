#pragma once

#include "BasicSettings.hpp"

// Store advanced settings
class AdvancedSettings : public BasicSettings {
public:
    AdvancedSettings(Parameters::AdvancedParameters& parameters,
                     const QString&                  groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Parameters::AdvancedParameters& m_parameters;
};
