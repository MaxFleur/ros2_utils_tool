#pragma once

#include "AdvancedSettings.hpp"

// Store bag editing parameters
class DeleteSourceSettings : public AdvancedSettings {
public:
    DeleteSourceSettings(Parameters::DeleteSourceParameters& parameters,
                         const QString&                      groupName);

    bool
    write() override;

protected:
    bool
    read() override;

private:
    Parameters::DeleteSourceParameters& m_parameters;
};
