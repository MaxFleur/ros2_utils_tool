#pragma once

#include "DialogSettings.hpp"
#include "Parameters.hpp"

#include <QDialog>
#include <QPointer>

class QCheckBox;

// Dialog used to modify settings
class SettingsDialog : public QDialog {
    Q_OBJECT

public:
    SettingsDialog(Parameters::DialogParameters& dialogParameters,
                   QWidget*                      parent = 0);

private:
    void
    storeParametersCheckStateChanged();

    void
    okClicked();

private:
    QPointer<QCheckBox> m_usePredefinedTopicNamesCheckBox;
    QPointer<QCheckBox> m_storeParametersCheckBox;
    QPointer<QCheckBox> m_checkROS2NamingConventionCheckBox;

    DialogSettings m_dialogSettings;
    Parameters::DialogParameters& m_dialogParameters;
};
