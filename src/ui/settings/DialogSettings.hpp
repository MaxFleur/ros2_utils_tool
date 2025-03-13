#pragma once

#include "GeneralSettings.hpp"
#include "Parameters.hpp"

// Settings modified from settings dialog
class DialogSettings : public GeneralSettings {
public:
    DialogSettings(Parameters::DialogParameters& parameters,
                   const QString&                groupName);

    // Make this static because we need to access the variable from many different places
    // in the application without wanting to use this as extra dependency
    [[nodiscard]] static bool
    areParametersSaved();

    static int
    maximumNumberOfThreads();

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::DialogParameters& m_parameters;
};
