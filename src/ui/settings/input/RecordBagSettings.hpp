#pragma once

#include "SelectableBagTopicSettings.hpp"

// Store bag recording parameters
class RecordBagSettings : public SelectableBagTopicSettings {
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
