#pragma once

#include "AdvancedSettings.hpp"

// Store bag editing parameters
class EditBagSettings : public AdvancedSettings {
public:
    EditBagSettings(Utils::UI::EditBagParameters& parameters,
                    const QString&                groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::EditBagParameters& m_parameters;
};
