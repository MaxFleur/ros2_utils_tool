#pragma once

#include "AdvancedSettings.hpp"

// Store tf2 to json parameters
class TF2ToFileSettings : public AdvancedSettings {
public:
    TF2ToFileSettings(Parameters::TF2ToFileParameters& parameters,
                      const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::TF2ToFileParameters& m_parameters;
};
