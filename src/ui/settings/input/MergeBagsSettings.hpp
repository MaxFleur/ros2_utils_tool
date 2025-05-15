#pragma once

#include "DeleteSourceSettings.hpp"

// Store bag editing parameters
class MergeBagsSettings : public DeleteSourceSettings {
public:
    MergeBagsSettings(Parameters::MergeBagsParameters& parameters,
                      const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::MergeBagsParameters& m_parameters;
};
