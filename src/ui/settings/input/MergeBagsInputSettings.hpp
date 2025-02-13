#pragma once

#include "AdvancedInputSettings.hpp"

// Store bag editing parameters
class MergeBagsInputSettings : public AdvancedInputSettings {
public:
    MergeBagsInputSettings(Utils::UI::MergeBagsInputParameters& parameters,
                           const QString&                       groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::MergeBagsInputParameters& m_parameters;
};
