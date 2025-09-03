#pragma once

#include "BagToVideoSettings.hpp"
#include "Parameters.hpp"
#include "TopicComboBoxWidget.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QFormLayout;

// Widget used to configure a video encoding out of a ros bag
class BagToVideoWidget : public TopicComboBoxWidget
{
    Q_OBJECT

public:
    BagToVideoWidget(Parameters::BagToVideoParameters& parameters,
                     QWidget*                          parent = 0);

private slots:
    void
    formatComboBoxTextChanged(const QString& text);

private:
    QPointer<QComboBox> m_formatComboBox;
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QCheckBox> m_useLosslessCheckBox;

    Parameters::BagToVideoParameters& m_parameters;

    BagToVideoSettings m_settings;
};
