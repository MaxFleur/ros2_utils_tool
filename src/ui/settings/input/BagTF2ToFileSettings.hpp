#pragma once

#include "AdvancedSettings.hpp"

// Store tf2 to file parameters
class BagTF2ToFileSettings : public AdvancedSettings {
public:
    BagTF2ToFileSettings(Parameters::BagTF2ToFileParameters& parameters,
                         const QString&                      groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::BagTF2ToFileParameters& m_parameters;
};
