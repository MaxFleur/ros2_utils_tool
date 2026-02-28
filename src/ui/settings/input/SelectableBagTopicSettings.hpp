#pragma once

#include "BasicSettings.hpp"

// Store topics for play bag and record bag settings
class SelectableBagTopicSettings : public BasicSettings {
public:
    SelectableBagTopicSettings(Parameters::SelectableBagTopicParameters& parameters,
                               const QString&                            groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Parameters::SelectableBagTopicParameters& m_parameters;
};
