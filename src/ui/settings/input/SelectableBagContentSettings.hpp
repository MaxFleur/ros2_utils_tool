#pragma once

#include "BasicSettings.hpp"

// Store parameters for bags with a selectable content
class SelectableBagContentSettings : public BasicSettings {
public:
    SelectableBagContentSettings(Parameters::SelectableBagContentParameters& parameters,
                                 const QString&                              groupName);
};
