#pragma once

#include <QMovie>
#include <QPointer>
#include <QWidget>

// Widget displaying a searching animation while information is being loaded in the background
class LoadingWidget : public QWidget
{
    Q_OBJECT
public:
    explicit
    LoadingWidget(QWidget* parent = 0);

    void
    startLoading()
    {
        m_movie->start();
        setVisible(true);
    }

    void
    stopLoading()
    {
        m_movie->stop();
        setVisible(false);
    }

private:
    bool
    event(QEvent *event) override;

private:
    QPointer<QMovie> m_movie;
};
