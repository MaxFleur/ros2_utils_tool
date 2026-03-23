#pragma once

#include "Parameters.hpp"

#include <QPointer>
#include <QToolButton>
#include <QWidget>

class QHBoxLayout;
class QListWidget;

// Widget used to control playing a bag file.
class ControlBagWidget : public QWidget
{
    Q_OBJECT

public:
    ControlBagWidget(Parameters::SelectableBagTopicParameters& parameters,
                     const QString&                            headerText,
                     const QString&                            headerPixmapLabelText,
                     bool                                      isRecorder,
                     QWidget*                                  parent = 0);

signals:
    void
    stopped();

protected slots:
    void
    setState();

protected:
    virtual void
    handleBagControlInstance()
    {
    }

    QToolButton*
    createButton(const QString& iconPath,
                 const QString& toolTipText,
                 int            buttonSize,
                 int            iconSize);

    void
    addLoggerWidgetEntry(const QString& entryText);

protected:
    QPointer<QHBoxLayout> m_controlsLayout;
    QPointer<QToolButton> m_playPauseButton;
    QPointer<QListWidget> m_loggerListWidget;

    bool m_isActive { true };
    bool m_isRecorder;

    static constexpr int TOOLBUTTON_SIZE = 40;
    static constexpr int TOOLBUTTON_ICON_SIZE = 20;
    static constexpr int TOOLBUTTON_SIZE_PLAYER = 70;
    static constexpr int TOOLBUTTON_ICON_SIZE_PLAYER = 40;
};
