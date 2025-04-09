#include "TopicListingInputWidget.hpp"

#include "UtilsUI.hpp"

#include <QEvent>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

TopicListingInputWidget::TopicListingInputWidget(Parameters::BasicParameters& parameters, const QString& titleText,
                                                 const QString& iconText, const QString& settingsIdentifierText,
                                                 QWidget *parent) :
    BasicInputWidget(titleText, iconText, parent),
    m_parameters(parameters), m_settings(parameters, settingsIdentifierText)
{
    m_removeTopicButton = new QToolButton;
    m_removeTopicButton->setToolTip("Remove the topic above.");
    m_addTopicButton = new QToolButton;
    m_addTopicButton->setToolTip("Add another topic.");

    m_topicButtonLayout = new QHBoxLayout;
    m_topicButtonLayout->addStretch();
    m_topicButtonLayout->addWidget(m_removeTopicButton);
    m_topicButtonLayout->addWidget(m_addTopicButton);

    m_controlsLayout = new QVBoxLayout;
    m_controlsLayout->addStretch();
    m_controlsLayout->addWidget(m_headerPixmapLabel);
    m_controlsLayout->addWidget(m_headerLabel);
    m_controlsLayout->addSpacing(40);

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(m_controlsLayout);
    controlsSqueezedLayout->addStretch();

    m_okButton->setEnabled(true);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    connect(m_findSourceButton, &QPushButton::clicked, this, &TopicListingInputWidget::sourceButtonPressed);
    connect(m_okButton, &QPushButton::clicked, this, &TopicListingInputWidget::okButtonPressed);
}


void
TopicListingInputWidget::sourceButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Bag File");
    if (fileName.isEmpty()) {
        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, fileName, m_settings);
    m_sourceLineEdit->setText(fileName);
}


void
TopicListingInputWidget::okButtonPressed()
{
    if (m_parameters.sourceDirectory.isEmpty()) {
        Utils::UI::createCriticalMessageBox("No bag name specified!", "Please specify a bag name before continuing!");
        return;
    }

    const auto isValid = areTopicsValid();
    if (isValid == std::nullopt) {
        return;
    }
    if (!*isValid) {
        Utils::UI::createCriticalMessageBox("Duplicate Topic Names Detected!", "Please make sure that no duplicate topic names are used!");
        return;
    }

    if (!Utils::UI::continueForExistingTarget(m_parameters.sourceDirectory, "Bagfile", "bag file")) {
        return;
    }

    emit okPressed();
}


void
TopicListingInputWidget::setPixmapLabelIcon()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? m_iconPath + "_white.svg" : m_iconPath + "_black.svg").pixmap(QSize(100, 45)));
    m_removeTopicButton->setIcon(QIcon(isDarkMode ? ":/icons/minus_white.svg" : ":/icons/minus_black.svg"));
    m_addTopicButton->setIcon(QIcon(isDarkMode ? ":/icons/plus_white.svg" : ":/icons/plus_black.svg"));
}


bool
TopicListingInputWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
