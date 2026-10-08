#pragma once

#include "SelectableBagContentSettings.hpp"

// Store play bag parameters
class PlayBagSettings : public SelectableBagContentSettings {
public:
    PlayBagSettings(Parameters::PlayBagParameters& parameters,
                    const QString&                 groupName);
};
