#pragma once

#include "AdvancedSettings.hpp"

// Store parameters to delete a source file
class DeleteSourceSettings : public AdvancedSettings {
public:
    DeleteSourceSettings(Parameters::DeleteSourceParameters& parameters,
                         const QString&                      groupName);
};
