#pragma once

#include <QPointer>
#include <QWidget>

class QToolButton;

// The starting widget showing all possible ui tools
class StartWidget : public QWidget
{
    Q_OBJECT
public:
    explicit
    StartWidget(QWidget* parent = 0);

signals:
    void
    bagToVideoRequested();

    void
    videoToBagRequested();

    void
    bagToImagesRequested();

    void
    playBagRequested();

private:
    QPointer<QToolButton>
    createToolButton(const QString& buttonText);

    void
    setButtonIcons();

    bool
    event(QEvent *event) override;

private:
    QPointer<QToolButton> m_bagToVideoPushButton;
    QPointer<QToolButton> m_bagToImagesPushButton;
    QPointer<QToolButton> m_videoToBagPushButton;
    QPointer<QToolButton> m_playBagButton;
};
