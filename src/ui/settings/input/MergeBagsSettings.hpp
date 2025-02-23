#pragma once

#include "AdvancedSettings.hpp"

// Store bag editing parameters
class MergeBagsSettings : public AdvancedSettings {
public:
    MergeBagsSettings(Utils::UI::MergeBagsParameters& parameters,
                      const QString&                  groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::MergeBagsParameters& m_parameters;
};
