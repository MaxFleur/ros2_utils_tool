#pragma once

#include "AdvancedSettings.hpp"

// Store bag editing parameters
class EditBagSettings : public AdvancedSettings {
public:
    EditBagSettings(Parameters::EditBagParameters& parameters,
                    const QString&                 groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::EditBagParameters& m_parameters;
};
