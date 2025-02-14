#pragma once

#include "BasicInputWidget.hpp"
#include "UtilsUI.hpp"
#include "VideoInputSettings.hpp"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QFormLayout;
class QLineEdit;

// Widget used to configure a video encoding out of a ros bag
class BagToVideoWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    BagToVideoWidget(Utils::UI::VideoInputParameters& parameters,
                     QWidget*                         parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    videoLocationButtonPressed();

    void
    formatComboBoxTextChanged(const QString& text);

    void
    okButtonPressed();

private:
    QPointer<QComboBox> m_topicNameComboBox;
    QPointer<QLineEdit> m_videoNameLineEdit;
    QPointer<QComboBox> m_formatComboBox;
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QCheckBox> m_useLosslessCheckBox;

    Utils::UI::VideoInputParameters& m_parameters;

    VideoInputSettings m_settings;
};
