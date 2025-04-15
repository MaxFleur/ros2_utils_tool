#pragma once

#include "BasicSettings.hpp"

// Store bag recording parameters
class RecordBagSettings : public BasicSettings {
public:
    RecordBagSettings(Parameters::RecordBagParameters& parameters,
                      const QString&                   groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::RecordBagParameters& m_parameters;
};
