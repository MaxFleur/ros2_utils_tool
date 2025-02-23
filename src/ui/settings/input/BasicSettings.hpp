#pragma once

#include "GeneralSettings.hpp"
#include "UtilsUI.hpp"

// Store parameters used by all input widgets
class BasicSettings : public GeneralSettings {
public:
    BasicSettings(Utils::UI::BasicParameters& parameters,
                  const QString&              groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Utils::UI::BasicParameters& m_parameters;
};
