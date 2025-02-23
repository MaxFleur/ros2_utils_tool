#pragma once

#include "BasicSettings.hpp"

// Store advanced settings
class AdvancedSettings : public BasicSettings {
public:
    AdvancedSettings(Utils::UI::AdvancedParameters& parameters,
                     const QString&                 groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::AdvancedParameters& m_parameters;
};
