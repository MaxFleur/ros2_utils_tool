#pragma once

#include "AdvancedSettings.hpp"

// Store parameters to delete a source file
class DeleteSourceSettings : public AdvancedSettings {
public:
    DeleteSourceSettings(Parameters::DeleteSourceParameters& parameters,
                         const QString&                      groupName);

    bool
    write() override;

protected:
    bool
    read() override;

    void
    setDefaultValueToTrue()
    {
        m_isDefaultValueTrue = true;
    }

private:
    Parameters::DeleteSourceParameters& m_parameters;

    bool m_isDefaultValueTrue = false;
};
