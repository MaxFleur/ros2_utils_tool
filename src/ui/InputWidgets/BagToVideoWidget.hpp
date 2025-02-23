#pragma once

#include "AdvancedInputWidget.hpp"
#include "BagToVideoSettings.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QFormLayout;

// Widget used to configure a video encoding out of a ros bag
class BagToVideoWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    BagToVideoWidget(Utils::UI::BagToVideoParameters& parameters,
                     QWidget*                         parent = 0);

private slots:
    void
    formatComboBoxTextChanged(const QString& text);

private:
    QPointer<QComboBox> m_formatComboBox;
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QCheckBox> m_useLosslessCheckBox;

    Utils::UI::BagToVideoParameters& m_parameters;

    BagToVideoSettings m_settings;
};
