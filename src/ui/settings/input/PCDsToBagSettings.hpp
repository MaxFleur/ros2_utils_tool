#pragma once

#include "AdvancedSettings.hpp"

// Store pcd to bag file tool parameters
class PCDsToBagSettings : public AdvancedSettings {
public:
    PCDsToBagSettings(Parameters::PCDsToBagParameters& parameters,
                      const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::PCDsToBagParameters& m_parameters;
};
