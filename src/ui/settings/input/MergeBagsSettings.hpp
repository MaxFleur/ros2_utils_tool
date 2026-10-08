#pragma once

#include "DeleteSourceSettings.hpp"

// Store merge bags parameters
class MergeBagsSettings : public DeleteSourceSettings {
public:
    MergeBagsSettings(Parameters::MergeBagsParameters& parameters,
                      const QString&                   groupName);
};
