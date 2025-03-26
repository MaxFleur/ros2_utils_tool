#pragma once

#include "DialogSettings.hpp"
#include "Parameters.hpp"

#include <QDialog>

// Dialog used to modify settings
class SettingsDialog : public QDialog {
    Q_OBJECT

public:
    SettingsDialog(Parameters::DialogParameters& parameters,
                   QWidget*                      parent = 0);

private:
    void
    storeParametersCheckStateChanged();

private:
    Parameters::DialogParameters& m_parameters;

    DialogSettings m_settings;
};
