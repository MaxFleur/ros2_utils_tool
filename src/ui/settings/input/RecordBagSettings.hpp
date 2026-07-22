#pragma once

#include "SelectableBagContentSettings.hpp"

// Store bag recording tool parameters
class RecordBagSettings : public SelectableBagContentSettings {
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
