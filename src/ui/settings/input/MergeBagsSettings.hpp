#pragma once

#include "AdvancedSettings.hpp"

// Store bag editing parameters
class MergeBagsSettings : public AdvancedSettings {
public:
    MergeBagsSettings(Parameters::MergeBagsParameters& parameters,
                      const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::MergeBagsParameters& m_parameters;
};
