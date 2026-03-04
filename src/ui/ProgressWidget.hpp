#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"
#include "UtilsUI.hpp"

#include <QPointer>
#include <QWidget>

class QMovie;

// Base widget showing overall progress
// The progress widget will access the main thread used to perform the corresponding operation.
// If the user presses the Cancel button, the thread will be cancelled and the input widget will be shown again.
class ProgressWidget : public QWidget
{
    Q_OBJECT

public:
    ProgressWidget(const QString&               headerLabelText,
                   Parameters::BasicParameters& parameters,
                   const Utils::UI::TOOL_ID     threadTypeId,
                   QWidget*                     parent = 0);

    ~ProgressWidget();

    void
    startThread()
    {
        m_thread->start();
    }

signals:
    void
    progressStopped();

    void
    finished();

private:
    bool
    event(QEvent *event) override;

private:
    QPointer<BasicThread> m_thread;

    QPointer<QMovie> m_movie;
};
