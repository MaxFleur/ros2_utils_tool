#pragma once

#include "AdvancedSettings.hpp"

// Store bag editing parameters
class BagToFileSettings : public AdvancedSettings {
public:
    BagToFileSettings(Parameters::BagToFileParameters& parameters,
                      const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::BagToFileParameters& m_parameters;
};
