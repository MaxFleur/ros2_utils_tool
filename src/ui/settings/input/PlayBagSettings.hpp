#pragma once

#include "SelectableBagContentSettings.hpp"

// Store play bag parameters
class PlayBagSettings : public SelectableBagContentSettings {
public:
    PlayBagSettings(Parameters::PlayBagParameters& parameters,
                    const QString&                 groupName);

    bool
    write() override;

private:
    bool
    read() override;

private:
    Parameters::PlayBagParameters& m_parameters;
};
