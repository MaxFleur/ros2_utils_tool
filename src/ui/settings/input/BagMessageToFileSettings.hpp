#pragma once

#include "AdvancedSettings.hpp"

// Store bag to yaml parameters
class BagMessageToFileSettings : public AdvancedSettings {
public:
    BagMessageToFileSettings(Parameters::BagMessageToFileParameters& parameters,
                             const QString&                          groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::BagMessageToFileParameters& m_parameters;
};
