#pragma once

#include "StoppableWidget.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QToolButton>
#include <QWidget>

class QHBoxLayout;
class QListWidget;
class QMovie;
class QVBoxLayout;

// Widget used to control playing a bag file.
class ControlBagWidget : public StoppableWidget
{
    Q_OBJECT

public:
    ControlBagWidget(Parameters::SelectableBagContentParameters& parameters,
                     const QString&                              headerText,
                     const QString&                              imagePath,
                     bool                                        isRecorder,
                     QWidget*                                    parent = 0);

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
    QPointer<QVBoxLayout> m_upperLayout;
    QPointer<QToolButton> m_playPauseButton;
    QPointer<QListWidget> m_loggerListWidget;

    QPointer<QMovie> m_movie;

    bool m_isActive { true };
    bool m_isRecorder;

    static constexpr int TOOLBUTTON_SIZE = 40;
    static constexpr int TOOLBUTTON_ICON_SIZE = 20;
    static constexpr int TOOLBUTTON_SIZE_PLAYER = 70;
    static constexpr int TOOLBUTTON_ICON_SIZE_PLAYER = 40;
};
