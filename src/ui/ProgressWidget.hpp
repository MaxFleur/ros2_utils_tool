#pragma once

#include "Parameters.hpp"

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
    ProgressWidget(const QString&               headerPixmapLabelTextBlack,
                   const QString&               headerPixmapLabelTextWhite,
                   const QString&               headerLabelText,
                   Parameters::BasicParameters& parameters,
                   const int                    threadTypeId,
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
    QPointer<BasicThread> m_thread;
};
