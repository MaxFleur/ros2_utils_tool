#pragma once

#include <QPointer>
#include <QWidget>

class QMovie;

// Widget displaying a searching animation while information is being loaded in the background
class LoadingWidget : public QWidget
{
    Q_OBJECT
public:
    explicit
    LoadingWidget(QWidget* parent = 0);

    void
    startLoading();

    void
    stopLoading();

private:
    bool
    event(QEvent *event) override;

private:
    QPointer<QMovie> m_movie;
};
