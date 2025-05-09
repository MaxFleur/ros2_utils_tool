#pragma once

#include "Parameters.hpp"
#include "UtilsUI.hpp"

#include <QMovie>
#include <QPointer>
#include <QWidget>

class BasicThread;

// Base widget showing overall progress
// The progress widget will access the main thread used to perform the corresponding operation.
// If the user presses the Cancel button, the thread will be cancelled and we'll return to the main window.
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
    startThread();

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
