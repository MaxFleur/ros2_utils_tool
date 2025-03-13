#pragma once

#include "DialogSettings.hpp"
#include "Parameters.hpp"

#include <QDialog>

// Dialog used to modify settings
class SettingsDialog : public QDialog {
    Q_OBJECT

public:
    SettingsDialog(Parameters::DialogParameters& dialogParameters,
                   QWidget*                      parent = 0);

private:
    void
    storeParametersCheckStateChanged();

private:
    DialogSettings m_dialogSettings;
    Parameters::DialogParameters& m_dialogParameters;
};
