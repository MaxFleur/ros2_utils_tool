#pragma once

#include "BasicInputWidget.hpp"
#include "Parameters.hpp"
#include "PublishSettings.hpp"

#include <QPointer>
#include <QWidget>

class QFormLayout;
class QLineEdit;
class QSpinBox;

// Widget used to configure publishing a video OR image sequence as ROS messages
class PublishWidget : public BasicInputWidget
{
    Q_OBJECT

public:
    PublishWidget(Parameters::PublishParameters& parameters,
                  bool                           usePredefinedTopicName,
                  bool                           checkROS2NameConform,
                  bool                           publishVideo,
                  QWidget*                       parent = 0);

private slots:
    void
    searchButtonPressed();

    void
    scaleCheckBoxPressed(int state);

    void
    okButtonPressed() const;

private:
    QPointer<QLineEdit> m_topicNameLineEdit;
    QPointer<QFormLayout> m_advancedOptionsFormLayout;
    QPointer<QSpinBox> m_widthSpinBox;
    QPointer<QSpinBox> m_heightSpinBox;

    Parameters::PublishParameters& m_parameters;

    PublishSettings m_settings;

    const bool m_checkROS2NameConform;
    const bool m_publishVideo;
};
