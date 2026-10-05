#pragma once

#include "DeleteSourceSettings.hpp"

// Store edit bag parameters
class EditBagSettings : public DeleteSourceSettings {
public:
    EditBagSettings(Parameters::EditBagParameters& parameters,
                    const QString&                 groupName);
};
