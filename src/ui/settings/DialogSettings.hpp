#pragma once

#include "GeneralSettings.hpp"
#include "UtilsUI.hpp"

// Settings modified from settings dialog
class DialogSettings : public GeneralSettings {
public:
    DialogSettings(Utils::UI::DialogParameters& parameters,
                   const QString&               groupName);

    // Make this static because we need to access the variable from many different places
    // in the application without wanting to use this as extra dependency
    [[nodiscard]] static bool
    areParametersSaved();

    bool
    write() override;

private:
    bool
    read() override;

private:
    Utils::UI::DialogParameters& m_parameters;
};
