#include "PlayBagProgressWidget.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QMovie>
#include <QPushButton>
#include <QVBoxLayout>

PlayBagProgressWidget::PlayBagProgressWidget(const Utils::UI::PlayBagParameters& playBagParameters, QWidget* parent) :
    QWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();

    auto* const headerPixmapLabel = new QLabel;
    headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/play_bag_white.svg" : ":/icons/play_bag_black.svg").pixmap(QSize(100, 45)));
    headerPixmapLabel->setAlignment(Qt::AlignHCenter);

    auto* const headerLabel = new QLabel("Playing ROSBag...");
    Utils::UI::setWidgetHeaderFont(headerLabel);
    headerLabel->setAlignment(Qt::AlignHCenter);

    auto* const gifLabel = new QLabel;
    QPointer<QMovie> const movie = new QMovie(isDarkMode ? ":gifs/playing_bag_white.gif" : ":gifs/playing_bag_black.gif");
    movie->setScaledSize(QSize(120, 100));
    gifLabel->setMovie(movie);
    gifLabel->setAlignment(Qt::AlignCenter);

    auto* const stopButton = new QPushButton("Stop");

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(stopButton);
    buttonLayout->addStretch();

    auto* const uiLayout = new QVBoxLayout;
    uiLayout->addStretch();
    uiLayout->addWidget(headerPixmapLabel);
    uiLayout->addWidget(headerLabel);
    uiLayout->addSpacing(30);
    uiLayout->addWidget(gifLabel);
    uiLayout->addStretch();

    auto* const uiLayoutStretched = new QHBoxLayout;
    uiLayoutStretched->addStretch();
    uiLayoutStretched->addLayout(uiLayout);
    uiLayoutStretched->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(uiLayoutStretched);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    setLayout(mainLayout);
    movie->start();

    m_thread = new PlayBagThread(playBagParameters);

    connect(stopButton, &QPushButton::clicked, this, &PlayBagProgressWidget::stopButtonPressed);
}


PlayBagProgressWidget::~PlayBagProgressWidget()
{
    m_thread->quit();
    m_thread->wait();
}


void
PlayBagProgressWidget::startThread()
{
    m_thread->start();
}


void
PlayBagProgressWidget::stopButtonPressed()
{
#ifdef ROS_JAZZY
    m_thread->stopPlaying();
    m_thread->quit();
    m_thread->wait();

    emit stopped(false);
#else
    /**
     * @note For ROS Humble there is no way to stop the player because there is no stop function
     *       and because it opens instance which cannot be quit. So we can only shutdown the application.
     */
    emit stopped(true);
#endif
}
