#pragma once

#include "BasicSettings.hpp"

// Store dummy bag creation parameters
class DummyBagSettings : public BasicSettings {
public:
    DummyBagSettings(Parameters::DummyBagParameters& parameters,
                     const QString&                  groupName);
};
