#include "ControlPlayBagWidget.hpp"

#include "UtilsUI.hpp"

#include <QHBoxLayout>
#include <QListWidget>
#include <QShortcut>
#include <QToolButton>

ControlPlayBagWidget::ControlPlayBagWidget(Parameters::PlayBagParameters& parameters, QWidget* parent)
    : ControlBagWidget(parameters, "Started playing Bag File\n", ":/icons/tools/play_bag_", false, parent), m_currentRate(parameters.rate)
{
    auto* const emptyWidget = new QWidget;
    emptyWidget->setFixedSize(QSize(TOOLBUTTON_SIZE, TOOLBUTTON_SIZE));

    const auto isDarkMode = Utils::UI::isDarkMode();
    auto* const decreaseRateButton = createButton(isDarkMode ? ":/icons/player/decrease_rate_white.svg" : ":/icons/player/decrease_rate_black.svg",
                                                  "Decrease playback rate by 0.1.", TOOLBUTTON_SIZE, TOOLBUTTON_ICON_SIZE);
    auto* const increaseRateButton = createButton(isDarkMode ? ":/icons/player/increase_rate_white.svg" : ":/icons/player/increase_rate_black.svg",
                                                  "Increase playback rate by 0.1.", TOOLBUTTON_SIZE, TOOLBUTTON_ICON_SIZE);
    auto* const playNextMessageButton = createButton(isDarkMode ? ":/icons/player/play_next_message_white.svg" : ":/icons/player/play_next_message_black.svg",
                                                     "Jump forward by one message.", TOOLBUTTON_SIZE, TOOLBUTTON_ICON_SIZE);

    m_controlsLayout->addWidget(emptyWidget);
    m_controlsLayout->addWidget(decreaseRateButton);
    m_controlsLayout->addWidget(m_playPauseButton);
    m_controlsLayout->addWidget(increaseRateButton);
    m_controlsLayout->addWidget(playNextMessageButton);
    m_controlsLayout->addStretch();

    auto* const decreaseRateShortcut = new QShortcut(QKeySequence(Qt::Key_Down), this);
    auto* const increaseRateShortcut = new QShortcut(QKeySequence(Qt::Key_Up), this);
    auto* const playNestMessageShortcut = new QShortcut(QKeySequence(Qt::Key_Right), this);

    connect(decreaseRateButton, &QToolButton::clicked, this, &ControlPlayBagWidget::decreaseRate);
    connect(increaseRateButton, &QToolButton::clicked, this, &ControlPlayBagWidget::increaseRate);
    connect(playNextMessageButton, &QToolButton::clicked, this, &ControlPlayBagWidget::playNextMessage);

    connect(decreaseRateShortcut, &QShortcut::activated, this, &ControlPlayBagWidget::decreaseRate);
    connect(increaseRateShortcut, &QShortcut::activated, this, &ControlPlayBagWidget::increaseRate);
    connect(playNestMessageShortcut, &QShortcut::activated, this, &ControlPlayBagWidget::playNextMessage);

    m_bagPlayer = std::make_unique<BagPlayer>(parameters);
    // Simulate the first few terminal info messages displayed when calling ros2 bag play
    addPlayBagLoggerEntry("Set rate to " + QString::number(m_currentRate));
    addPlayBagLoggerEntry("Press SPACE for Pause/Resume");
    addPlayBagLoggerEntry("Press CURSOR_RIGHT for Play Next Message");
    addPlayBagLoggerEntry("Press CURSOR_UP for Increase Rate 10%");
    addPlayBagLoggerEntry("Press CURSOR_DOWN for Decrease Rate 10%");

    QString topicsString = "";
    const auto addTopics = [&topicsString] (const QVector<Parameters::SelectableBagContent>& params) {
        for (auto i = 0; i < params.size(); ++i) {
            if (!params.at(i).isSelected) {
                continue;
            }

            topicsString += params.at(i).name + " ";
        }
    };

    addTopics(parameters.topics);
    addTopics(parameters.services);

    addPlayBagLoggerEntry("Playing topics " + topicsString);
    addPlayBagLoggerEntry("Started play.");
    m_loggerListWidget->setFocus();
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
    addPlayBagLoggerEntry("Set rate to " + QString::number(m_currentRate));
}


void
ControlPlayBagWidget::increaseRate()
{
    m_currentRate += 0.1;
    m_bagPlayer->setRate(m_currentRate);
    addPlayBagLoggerEntry("Set rate to " + QString::number(m_currentRate));
}


void
ControlPlayBagWidget::playNextMessage()
{
    m_bagPlayer->playNextMessage();
    addPlayBagLoggerEntry("Playing next message.");
}
