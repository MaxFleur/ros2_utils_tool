#pragma once

#include "AdvancedSettings.hpp"

// Store bag to yaml parameters
class BagToYamlSettings : public AdvancedSettings {
public:
    BagToYamlSettings(Parameters::BagToYamlParameters& parameters,
                      const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::BagToYamlParameters& m_parameters;
};
