#pragma once

#include "SelectableBagContentSettings.hpp"

// Store bag recording tool parameters
class RecordBagSettings : public SelectableBagContentSettings {
public:
    RecordBagSettings(Parameters::RecordBagParameters& parameters,
                      const QString&                   groupName);
};
