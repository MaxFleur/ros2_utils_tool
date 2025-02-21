#pragma once

#include "AdvancedInputWidget.hpp"
#include "UtilsUI.hpp"
#include "VideoInputSettings.hpp"

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
    BagToVideoWidget(Utils::UI::VideoInputParameters& parameters,
                     QWidget*                         parent = 0);

private slots:
    void
    formatComboBoxTextChanged(const QString& text);

private:
    QPointer<QComboBox> m_formatComboBox;
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QCheckBox> m_useLosslessCheckBox;

    Utils::UI::VideoInputParameters& m_parameters;

    VideoInputSettings m_settings;
};
