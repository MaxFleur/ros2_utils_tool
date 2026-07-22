#pragma once

#include <QWidget>

class QMovie;

// A simple widget which has only a stopped signal.
// Used by the control bag and progress widget to stop certain ungoing operations.
class StoppableWidget : public QWidget
{
    Q_OBJECT

public:
    StoppableWidget(QWidget* parent = 0) : QWidget(parent)
    {
    }

signals:
    void
    stopped();

    void
    finished();
};
