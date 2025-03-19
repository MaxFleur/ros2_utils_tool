#pragma once

#include "AdvancedSettings.hpp"

// Store bag editing parameters
class CompressBagSettings : public AdvancedSettings {
public:
    CompressBagSettings(Parameters::CompressBagParameters& parameters,
                        const QString&                     groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::CompressBagParameters& m_parameters;
};
