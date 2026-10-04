#pragma once

#include "BagTF2ToFileSettings.hpp"
#include "TopicComboBoxWidget.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class QComboBox;
class QFormLayout;
class QRadioButton;

// Widget used to configure writing a ROS bag tf message topic to a json/yaml file
class BagTF2ToFileWidget : public TopicComboBoxWidget
{
    Q_OBJECT

public:
    BagTF2ToFileWidget(Parameters::BagTF2ToFileParameters& parameters,
                       QWidget*                            parent = 0);

    void
    formatComboBoxTextChanged(bool switched);

private:
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QRadioButton> m_compactRadioButton;
    QPointer<QRadioButton> m_indentedRadioButton;
    QPointer<QComboBox> m_formatComboBox;

    Parameters::BagTF2ToFileParameters& m_parameters;

    BagTF2ToFileSettings m_settings;
};
