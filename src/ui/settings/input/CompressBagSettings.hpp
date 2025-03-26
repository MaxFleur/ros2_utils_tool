#pragma once

#include "DeleteSourceSettings.hpp"

// Store bag editing parameters
class CompressBagSettings : public DeleteSourceSettings {
public:
    CompressBagSettings(Parameters::CompressBagParameters& parameters,
                        const QString&                     groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::CompressBagParameters& m_parameters;
};
