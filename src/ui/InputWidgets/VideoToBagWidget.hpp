#pragma once

#include "AdvancedInputWidget.hpp"
#include "Parameters.hpp"
#include "VideoToBagSettings.hpp"

#include <QPointer>
#include <QWidget>

class QFormLayout;
class QSpinBox;

// Widget used to write a video file into a ROSBag
class VideoToBagWidget : public AdvancedInputWidget
{
    Q_OBJECT

public:
    VideoToBagWidget(Parameters::VideoToBagParameters& parameters,
                     bool                              usePredefinedTopicName,
                     bool                              checkROS2NameConform,
                     QWidget*                          parent = 0);

private slots:
    void
    searchButtonPressed() override;

    void
    useCustomFPSCheckBoxPressed(int state);

    void
    okButtonPressed() override;

private:
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QSpinBox> m_fpsSpinBox;

    Parameters::VideoToBagParameters& m_parameters;

    VideoToBagSettings m_settings;

    const bool m_checkROS2NameConform;
};
