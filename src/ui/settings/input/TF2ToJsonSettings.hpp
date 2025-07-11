#pragma once

#include "AdvancedSettings.hpp"

// Store tf2 to json parameters
class TF2ToJsonSettings : public AdvancedSettings {
public:
    TF2ToJsonSettings(Parameters::TF2ToJsonParameters& parameters,
                      const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::TF2ToJsonParameters& m_parameters;
};
