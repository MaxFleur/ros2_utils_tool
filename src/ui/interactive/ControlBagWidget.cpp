#include "ControlBagWidget.hpp"

#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMovie>
#include <QPushButton>
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

ControlBagWidget::ControlBagWidget(Parameters::SelectableBagContentParameters& parameters,
                                   const QString& headerText, const QString& moviePath, bool isRecorder, QWidget* parent)
    : StoppableWidget(parent), m_isRecorder(isRecorder)
{
    // Potentially cropped string showing the target file to be recorded/played
    auto targetFileString = parameters.sourceDirectory;
    if (targetFileString.length() > 40) {
        targetFileString = "..." + targetFileString.right(40);
    }

    auto* const headerLabel = new QLabel("<b>" + headerText + "</b><br>" + targetFileString);
    auto font = headerLabel->font();
    font.setPointSize(12);
    headerLabel->setFont(font);
    headerLabel->setAlignment(Qt::AlignHCenter);

    const auto isDarkMode = Utils::UI::isDarkMode();

    m_movie = new QMovie(isDarkMode ? moviePath + "_white.gif" : moviePath + "_black.gif");
    m_movie->setScaledSize(QSize(120, 70));

    auto* const movieLabel = new QLabel;
    movieLabel->setMovie(m_movie);
    movieLabel->setAlignment(Qt::AlignHCenter);

    const QString activeString = m_isRecorder ? "Record" : "Play";
    m_playPauseButton = createButton(isDarkMode ? ":/icons/player/stop_white.svg" : ":/icons/player/stop_black.svg",
                                     activeString + " or pause.", TOOLBUTTON_SIZE_PLAYER, TOOLBUTTON_ICON_SIZE_PLAYER);

    m_controlsLayout = new QHBoxLayout;
    m_controlsLayout->addStretch();

    m_loggerListWidget = new QListWidget;
    m_loggerListWidget->setMinimumHeight(200);

    auto* const stopButton = new QPushButton("Stop");
    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(stopButton);
    buttonLayout->setAlignment(stopButton, Qt::AlignLeft);

    m_upperLayout = new QVBoxLayout;
    m_upperLayout->addStretch();
    m_upperLayout->addWidget(headerLabel);
    m_upperLayout->addSpacing(20);
    m_upperLayout->addWidget(movieLabel);
    m_upperLayout->addLayout(m_controlsLayout);
    m_upperLayout->addSpacing(10);
    m_upperLayout->addWidget(m_loggerListWidget);
    m_upperLayout->setContentsMargins(20, 20, 20, 20);
    m_upperLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(m_upperLayout);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    auto* const playPauseShortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
    auto* const stopPlayShortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);

    connect(m_playPauseButton, &QToolButton::clicked, this, &ControlBagWidget::setState);
    connect(stopButton, &QPushButton::clicked, this, [this] {
        emit stopped();
    });

    connect(playPauseShortcut, &QShortcut::activated, this, &ControlBagWidget::setState);
    connect(stopPlayShortcut, &QShortcut::activated, this, [this] {
        emit stopped();
    });

    m_movie->start();
}


void
ControlBagWidget::setState()
{
    m_isActive = !m_isActive;
    handleBagControlInstance();

    const QString recordOrPlayString = m_isRecorder ? "recording" : "play";
    const auto activeString = m_isActive ? "Resuming " + recordOrPlayString + "." : "Pausing " + recordOrPlayString + ".";
    addLoggerWidgetEntry(Utils::ROS::getCurrentROSTimeAsString() + " " + activeString);

    // Button icon has to switch between "Pause" and "Play"
    const auto isDarkMode = Utils::UI::isDarkMode();
    if (m_isActive) {
        m_playPauseButton->setIcon(QIcon(isDarkMode ? ":/icons/player/stop_white.svg" : ":/icons/player/stop_black.svg"));
        m_movie->start();
        return;
    }
    m_playPauseButton->setIcon(QIcon(isDarkMode ? ":/icons/player/play_white.svg" : ":/icons/player/play_black.svg"));
    m_movie->stop();
}


QToolButton*
ControlBagWidget::createButton(const QString& iconPath, const QString& toolTipText, int buttonSize, int iconSize)
{
    auto* const button = new QToolButton;
    button->setFixedSize(QSize(buttonSize, buttonSize));
    button->setIcon(QIcon(iconPath));
    button->setToolTip(toolTipText);
    button->setIconSize(QSize(iconSize, iconSize));

    return button;
}


void
ControlBagWidget::addLoggerWidgetEntry(const QString& entryText)
{
    // Give it a ROS terminal log like style
    auto* const item = new QListWidgetItem(entryText);
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);

    m_loggerListWidget->addItem(item);
    m_loggerListWidget->setCurrentRow(m_loggerListWidget->count() - 1);
}
