#pragma once

#include "BagPlayer.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

class QListWidget;
class QToolButton;

// Widget used control playing a bag file.
class ControlPlayBagWidget : public QWidget
{
    Q_OBJECT

public:
    ControlPlayBagWidget(Parameters::PlayBagParameters& parameters,
                         QWidget*                       parent = 0);

signals:
    void
    stopped();

private slots:
    void
    setPlayerState();

    void
    decreaseRate();

    void
    increaseRate();

    void
    playNextMessage();

private:
    void
    addLoggerWidgetEntry(const QString& entryText);

private:
    std::unique_ptr<BagPlayer> m_bagPlayer;

    QPointer<QListWidget> m_loggerWidget;
    QPointer<QToolButton> m_playPauseButton;

    double m_currentRate;
    bool m_isPlaying { true };

    static constexpr int TOOLBUTTON_SIZE = 40;
    static constexpr int TOOLBUTTON_ICON_SIZE = 20;
    static constexpr int TOOLBUTTON_SIZE_PLAYER = 70;
    static constexpr int TOOLBUTTON_ICON_SIZE_PLAYER = 40;
};
