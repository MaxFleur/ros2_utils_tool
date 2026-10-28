#include "LoadingWidget.hpp"

#include "UtilsUI.hpp"

#include <QEvent>
#include <QLabel>
#include <QVBoxLayout>

LoadingWidget::LoadingWidget(QWidget* parent) : QWidget(parent)
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_movie = new QMovie(Utils::UI::themedIconPath(":/gifs/searching", ".gif", isDarkMode));
    m_movie->setScaledSize(QSize(100, 100));

    auto* const movieLabel = new QLabel;
    movieLabel->setMovie(m_movie);
    movieLabel->setAlignment(Qt::AlignHCenter);

    auto* const searchingLabel = new QLabel("<b>Searching for Information...</b>");

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addWidget(movieLabel);
    mainLayout->addSpacing(10);
    mainLayout->addWidget(searchingLabel);
    mainLayout->setAlignment(movieLabel, Qt::AlignCenter);
    mainLayout->setAlignment(searchingLabel, Qt::AlignCenter);
    setLayout(mainLayout);

    setVisible(false);
}


bool
LoadingWidget::event(QEvent *event)
{
    [[unlikely]] if ((event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange)) {
        const auto isDarkMode = Utils::UI::isDarkMode();
        m_movie->setFileName(Utils::UI::themedIconPath(":/gifs/searching", ".gif", isDarkMode));
        m_movie->start();
    }
    return QWidget::event(event);
}
