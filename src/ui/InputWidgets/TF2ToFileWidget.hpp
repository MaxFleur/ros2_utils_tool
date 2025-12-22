#pragma once

#include "TF2ToFileSettings.hpp"
#include "TopicComboBoxWidget.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class QComboBox;
class QFormLayout;
class QRadioButton;

// Widget used to configure sending tf2 messages
class TF2ToFileWidget : public TopicComboBoxWidget
{
    Q_OBJECT

public:
    TF2ToFileWidget(Parameters::TF2ToFileParameters& parameters,
                    QWidget*                         parent = 0);

    void
    formatComboBoxTextChanged(bool switched);

private:
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QRadioButton> m_compactRadioButton;
    QPointer<QRadioButton> m_indentedRadioButton;
    QPointer<QComboBox> m_formatComboBox;

    Parameters::TF2ToFileParameters& m_parameters;

    TF2ToFileSettings m_settings;
};
