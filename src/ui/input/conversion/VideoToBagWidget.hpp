#pragma once

#include "Parameters.hpp"
#include "TopicComboBoxWidget.hpp"
#include "VideoToBagSettings.hpp"

#include <QPointer>
#include <QWidget>

class QFormLayout;
class QSpinBox;
class QRadioButton;

// Widget used to write a video file to a ROS bag file
class VideoToBagWidget : public TopicComboBoxWidget
{
    Q_OBJECT

public:
    VideoToBagWidget(Parameters::VideoToBagParameters& parameters,
                     bool                              usePredefinedTopicName,
                     bool                              warnROS2NameConvention,
                     QWidget*                          parent = 0);

private slots:
    void
    findSourceButtonPressed() override;

    void
    useCustomFPSCheckBoxPressed(int state);

    void
    useCompressionCheckBoxPressed(int state);

    void
    okButtonPressed() const override;

private:
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QSpinBox> m_fpsSpinBox;
    QPointer<QRadioButton> m_compressJPEGRadioButton;
    QPointer<QRadioButton> m_compressPNGRadioButton;

    Parameters::VideoToBagParameters& m_parameters;

    VideoToBagSettings m_settings;

    const bool m_warnROS2NameConvention;

    static constexpr int ADVANCED_OPTIONS_LAYOUT_ROW_THIRD = 2;
    static constexpr int ADVANCED_OPTIONS_LAYOUT_ROW_FOURTH = 3;
    static constexpr int ADVANCED_OPTIONS_LAYOUT_ROW_FIFTH = 4;
    static constexpr int ADVANCED_OPTIONS_LAYOUT_ROW_SIXTH = 5;
};
