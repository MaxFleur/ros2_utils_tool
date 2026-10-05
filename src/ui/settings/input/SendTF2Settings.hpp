#pragma once

#include "BasicSettings.hpp"
#include "Parameters.hpp"

// Store sending tf2 parameters
class SendTF2Settings : public BasicSettings {
public:
    SendTF2Settings(Parameters::SendTF2Parameters& parameters,
                    const QString&                 groupName);
};
