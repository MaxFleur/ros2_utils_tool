#pragma once

#include "BasicSettings.hpp"

// Store topics for play bag and record bag parameters
class SelectableBagContentSettings : public BasicSettings {
public:
    SelectableBagContentSettings(Parameters::SelectableBagContentParameters& parameters,
                                 const QString&                              groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Parameters::SelectableBagContentParameters& m_parameters;
};
