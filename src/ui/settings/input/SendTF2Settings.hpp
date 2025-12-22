#pragma once

#include "BasicSettings.hpp"
#include "Parameters.hpp"

// Store sending tf2 parameters
class SendTF2Settings : public BasicSettings {
public:
    SendTF2Settings(Parameters::SendTF2Parameters& parameters,
                    const QString&                 groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::SendTF2Parameters& m_parameters;

    static constexpr int TRANSLATION_SIZE = 3;
    static constexpr int ROTATION_SIZE = 4;
};
