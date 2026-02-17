#include "ControlPlayBagWidget.hpp"

#include "UtilsUI.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QShortcut>
#include <QToolButton>
#include <QVBoxLayout>

ControlPlayBagWidget::ControlPlayBagWidget(Parameters::PlayBagParameters& parameters, QWidget* parent)
    : QWidget(parent), m_currentRate(parameters.rate)
{
    auto* const headerLabel = new QLabel("Started playing Bag File\n" + parameters.sourceDirectory);
    auto font = headerLabel->font();
    font.setBold(true);
    headerLabel->setFont(font);
    headerLabel->setAlignment(Qt::AlignHCenter);

    auto* const headerPixmapLabel = new QLabel;
    headerPixmapLabel->setAlignment(Qt::AlignHCenter);

    const auto isDarkMode = Utils::UI::isDarkMode();
    headerPixmapLabel->setPixmap(QIcon(isDarkMode ? ":/icons/play_bag_white.svg" : ":/icons/play_bag_black.svg").pixmap(QSize(100, 45)));

    auto* const emptyWidget = new QWidget;
    emptyWidget->setFixedSize(QSize(TOOLBUTTON_SIZE, TOOLBUTTON_SIZE));

    const auto createButton = [] (int buttonSize, int iconSize, const QString& iconPath, const QString& toolTipText) {
        auto* const button = new QToolButton;
        button->setFixedSize(QSize(buttonSize, buttonSize));
        button->setIcon(QIcon(iconPath));
        button->setToolTip(toolTipText);
        button->setIconSize(QSize(iconSize, iconSize));

        return button;
    };

    auto* const decreaseRateButton = createButton(TOOLBUTTON_SIZE, TOOLBUTTON_ICON_SIZE,
                                                  isDarkMode ? ":/icons/player/decrease_rate_white.svg" : ":/icons/player/decrease_rate_black.svg",
                                                  "Decrease playback rate by 0.1.");
    m_playPauseButton = createButton(TOOLBUTTON_SIZE_PLAYER, TOOLBUTTON_ICON_SIZE_PLAYER,
                                     isDarkMode ? ":/icons/player/stop_white.svg" : ":/icons/player/stop_black.svg",
                                     "Play or pause.");
    auto* const increaseRateButton = createButton(TOOLBUTTON_SIZE, TOOLBUTTON_ICON_SIZE,
                                                  isDarkMode ? ":/icons/player/increase_rate_white.svg" : ":/icons/player/increase_rate_black.svg",
                                                  "Increase playback rate by 0.1.");
    auto* const playNextMessageButton = createButton(TOOLBUTTON_SIZE, TOOLBUTTON_ICON_SIZE,
                                                     isDarkMode ? ":/icons/player/play_next_message_white.svg" : ":/icons/player/play_next_message_black.svg",
                                                     "Jump forward by one message.");

    auto* const controlsLayout = new QHBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(emptyWidget);
    controlsLayout->addWidget(decreaseRateButton);
    controlsLayout->addWidget(m_playPauseButton);
    controlsLayout->addWidget(increaseRateButton);
    controlsLayout->addWidget(playNextMessageButton);
    controlsLayout->addStretch();

    m_loggerWidget = new QListWidget;
    m_loggerWidget->setMinimumHeight(200);

    auto* const stopButton = new QPushButton("Stop");
    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(stopButton);
    buttonLayout->setAlignment(stopButton, Qt::AlignLeft);

    auto* const upperLayout = new QVBoxLayout;
    upperLayout->addStretch();
    upperLayout->addWidget(headerPixmapLabel);
    upperLayout->addWidget(headerLabel);
    upperLayout->addSpacing(40);
    upperLayout->addLayout(controlsLayout);
    upperLayout->addSpacing(10);
    upperLayout->addWidget(m_loggerWidget);
    upperLayout->setContentsMargins(20, 20, 20, 20);
    upperLayout->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(upperLayout);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    auto* const playPauseShortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
    auto* const decreaseRateShortcut = new QShortcut(QKeySequence(Qt::Key_Down), this);
    auto* const increaseRateShortcut = new QShortcut(QKeySequence(Qt::Key_Up), this);
    auto* const playNestMessageShortcut = new QShortcut(QKeySequence(Qt::Key_Right), this);
    auto* const stopPlayShortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);

    connect(m_playPauseButton, &QToolButton::clicked, this, &ControlPlayBagWidget::setPlayerState);
    connect(decreaseRateButton, &QToolButton::clicked, this, &ControlPlayBagWidget::decreaseRate);
    connect(increaseRateButton, &QToolButton::clicked, this, &ControlPlayBagWidget::increaseRate);
    connect(playNextMessageButton, &QToolButton::clicked, this, &ControlPlayBagWidget::playNextMessage);
    connect(stopButton, &QPushButton::clicked, this, [this] {
        emit stopped();
    });

    connect(playPauseShortcut, &QShortcut::activated, this, &ControlPlayBagWidget::setPlayerState);
    connect(decreaseRateShortcut, &QShortcut::activated, this, &ControlPlayBagWidget::decreaseRate);
    connect(increaseRateShortcut, &QShortcut::activated, this, &ControlPlayBagWidget::increaseRate);
    connect(playNestMessageShortcut, &QShortcut::activated, this, &ControlPlayBagWidget::playNextMessage);
    connect(stopPlayShortcut, &QShortcut::activated, this, [this] {
        emit stopped();
    });

    m_bagPlayer = std::make_unique<BagPlayer>(parameters);
    // Simulate the first few terminal info messages when calling ros2 bag play
    addLoggerWidgetEntry("Set rate to " + QString::number(m_currentRate));
    addLoggerWidgetEntry("Press SPACE for Pause/Resume");
    addLoggerWidgetEntry("Press CURSOR_RIGHT for Play Next Message");
    addLoggerWidgetEntry("Press CURSOR_UP for Increase Rate 10%");
    addLoggerWidgetEntry("Press CURSOR_DOWN for Decrease Rate 10%");

    QString topicsString = "";
    for (auto i = 0; i < parameters.topics.size(); ++i) {
        if (!parameters.topics.at(i).isSelected) {
            continue;
        }

        topicsString += parameters.topics.at(i).name + " ";
    }
    addLoggerWidgetEntry("Playing topics " + topicsString);
    addLoggerWidgetEntry("Started play.");
    m_loggerWidget->setFocus();
}


void
ControlPlayBagWidget::setPlayerState()
{
    m_isPlaying = !m_isPlaying;
    m_bagPlayer->togglePlayerState(m_isPlaying);

    const auto isDarkMode = Utils::UI::isDarkMode();
    if (m_isPlaying) {
        m_playPauseButton->setIcon(QIcon(isDarkMode ? ":/icons/player/stop_white.svg" : ":/icons/player/stop_black.svg"));
        addLoggerWidgetEntry("Resuming play.");
        return;
    }
    m_playPauseButton->setIcon(QIcon(isDarkMode ? ":/icons/player/play_white.svg" : ":/icons/player/play_black.svg"));
    addLoggerWidgetEntry("Pausing play.");
}


void
ControlPlayBagWidget::decreaseRate()
{
    m_currentRate -= 0.1;
    // 0.1 - 0.1 is not 0, as we know from floating points
    if (std::abs(m_currentRate) < 1e-9) {
        m_currentRate = 0.0;
    }
    m_currentRate = std::max(m_currentRate, 0.0);

    m_bagPlayer->setRate(m_currentRate);
    addLoggerWidgetEntry("Set rate to " + QString::number(m_currentRate));
}


void
ControlPlayBagWidget::increaseRate()
{
    m_currentRate += 0.1;
    m_bagPlayer->setRate(m_currentRate);
    addLoggerWidgetEntry("Set rate to " + QString::number(m_currentRate));
}


void
ControlPlayBagWidget::playNextMessage()
{
    m_bagPlayer->playNextMessage();
    addLoggerWidgetEntry("Playing next message.");
}


void
ControlPlayBagWidget::addLoggerWidgetEntry(const QString& entryText)
{
    auto* const item = new QListWidgetItem("[" + m_bagPlayer->getCurrentTimeAsString() + "] " + entryText);
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);

    m_loggerWidget->addItem(item);
    m_loggerWidget->setCurrentRow(m_loggerWidget->count() - 1);
}
