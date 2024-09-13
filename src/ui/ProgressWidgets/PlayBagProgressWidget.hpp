#pragma once

#include "PlayBagThread.hpp"
#include "UtilsUI.hpp"

#include <QWidget>

/**
 * @brief Progress widget variant used to display that a ROSBag is played right now
 */
class PlayBagProgressWidget : public QWidget
{
    Q_OBJECT

public:
    PlayBagProgressWidget(const Utils::UI::PlayBagParameters& playBagParameters,
                          QWidget*                            parent = 0);

    void
    startThread();

signals:
    void
    stopped(bool terminateApplication);

private slots:
    void
    stopButtonPressed();

private:
    QPointer<PlayBagThread> m_thread;
};
