#pragma once

#include "BagInputSettings.hpp"
#include "BasicInputWidget.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QFormLayout;
class QLineEdit;
class QSpinBox;

// Widget used to write a video file into a ROSBag
class VideoToBagWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    VideoToBagWidget(Utils::UI::BagInputParameters& parameters,
                     bool                           usePredefinedTopicName,
                     bool                           checkROS2NameConform,
                     QWidget*                       parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    bagLocationButtonPressed();

    void
    useCustomFPSCheckBoxPressed(int state);

    void
    okButtonPressed();

private:
    QPointer<QLineEdit> m_bagNameLineEdit;
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QSpinBox> m_fpsSpinBox;

    Utils::UI::BagInputParameters& m_parameters;

    BagInputSettings m_settings;

    const bool m_checkROS2NameConform;
};
