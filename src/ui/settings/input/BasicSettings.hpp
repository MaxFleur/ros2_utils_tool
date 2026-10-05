#pragma once

#include "GeneralSettings.hpp"
#include "Parameters.hpp"

// Store parameters used by all input widgets
class BasicSettings : public GeneralSettings {
public:
    BasicSettings(Parameters::BasicParameters& parameters,
                  const QString&               groupName);
};
