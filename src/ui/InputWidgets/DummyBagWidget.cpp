#include "DummyBagWidget.hpp"

#include "DummyTopicWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QEvent>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSet>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

#include <filesystem>

DummyBagWidget::DummyBagWidget(Parameters::DummyBagParameters& parameters, bool checkROS2NameConform, QWidget *parent) :
    BasicInputWidget("Create Dummy Bag", ":/icons/dummy_bag", parent),
    m_parameters(parameters), m_settings(parameters, "dummy_bag"),
    m_checkROS2NameConform(checkROS2NameConform)
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The target dummy bag file directory.");

    auto* const messageCountSpinBox = new QSpinBox;
    messageCountSpinBox->setRange(1, 1000);
    messageCountSpinBox->setToolTip("The number of messages stored in the bag.");
    messageCountSpinBox->setValue(m_parameters.messageCount);

    m_minusButton = new QToolButton;
    m_minusButton->setToolTip("Remove the topic above.");
    m_plusButton = new QToolButton;
    m_plusButton->setToolTip("Add another topic. Currently, a maximum of four topics can be added.");

    auto* const plusMinusButtonLayout = new QHBoxLayout;
    plusMinusButtonLayout->addStretch();
    plusMinusButtonLayout->addWidget(m_minusButton);
    plusMinusButtonLayout->addWidget(m_plusButton);

    m_formLayout = new QFormLayout;
    m_formLayout->addRow("Target Bag Location:", m_findSourceLayout);
    m_formLayout->addRow("", plusMinusButtonLayout);
    m_formLayout->addRow("Message Count:", messageCountSpinBox);

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(40);
    controlsLayout->addLayout(m_formLayout);
    controlsLayout->addSpacing(20);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    m_okButton->setEnabled(true);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    const auto addNewTopic = [this] {
        m_parameters.topics.push_back({ "String", "" });
        m_settings.write();
        createNewDummyTopicWidget({ "", "" }, m_parameters.topics.size() - 1);
    };
    // Create widgets for already existing topics
    for (auto i = 0; i < m_parameters.topics.size(); i++) {
        createNewDummyTopicWidget(m_parameters.topics.at(i), i);
    }
    if (m_parameters.topics.empty()) {
        addNewTopic();
    }

    connect(m_findSourceButton, &QPushButton::clicked, this, &DummyBagWidget::bagDirectoryButtonPressed);
    connect(messageCountSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        writeParameterToSettings(m_parameters.messageCount, value, m_settings);
    });
    connect(m_minusButton, &QPushButton::clicked, this, &DummyBagWidget::removeDummyTopicWidget);
    connect(m_plusButton, &QPushButton::clicked, this, [addNewTopic] {
        addNewTopic();
    });
    connect(m_okButton, &QPushButton::clicked, this, &DummyBagWidget::okButtonPressed);

    setPixmapLabelIcon();
}


void
DummyBagWidget::bagDirectoryButtonPressed()
{
    const auto fileName = QFileDialog::getSaveFileName(this, "Save Dummy Bag File");
    if (fileName.isEmpty()) {
        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, fileName, m_settings);
    m_sourceLineEdit->setText(fileName);
}


void
DummyBagWidget::removeDummyTopicWidget()
{
    m_formLayout->removeRow(m_parameters.topics.size());
    m_dummyTopicWidgets.pop_back();
    m_parameters.topics.pop_back();
    m_settings.write();
    m_numberOfTopics--;

    m_plusButton->setEnabled(m_numberOfTopics != MAXIMUM_NUMBER_OF_TOPICS);
    m_minusButton->setEnabled(m_parameters.topics.size() != 1);
}


void
DummyBagWidget::createNewDummyTopicWidget(const Parameters::DummyBagParameters::DummyBagTopic& topic, int index)
{
    auto* const dummyTopicWidget = new DummyTopicWidget(topic.type, topic.name);

    connect(dummyTopicWidget, &DummyTopicWidget::topicTypeChanged, this, [this, index] (const QString& text) {
        writeParameterToSettings(m_parameters.topics[index].type, text, m_settings);
    });
    connect(dummyTopicWidget, &DummyTopicWidget::topicNameChanged, this, [this, index] (const QString& text) {
        writeParameterToSettings(m_parameters.topics[index].name, text, m_settings);
    });

    // Keep it all inside the main form layout
    m_formLayout->insertRow(m_formLayout->rowCount() - 2, "Topic " + QString::number(m_numberOfTopics + 1) + ":", dummyTopicWidget);
    m_dummyTopicWidgets.push_back(dummyTopicWidget);

    m_numberOfTopics++;
    m_plusButton->setEnabled(m_numberOfTopics != MAXIMUM_NUMBER_OF_TOPICS);
    m_minusButton->setEnabled(m_parameters.topics.size() != 1);
}


void
DummyBagWidget::okButtonPressed()
{
    if (m_parameters.sourceDirectory.isEmpty()) {
        Utils::UI::createCriticalMessageBox("No bag name specified!", "Please specify a bag name before continuing!");
        return;
    }

    auto areROS2NamesValid = true;
    // Sets remove duplicates, so use a set to check if duplicate topic names exist
    QSet<QString> topicNameSet;
    for (QPointer<DummyTopicWidget> dummyTopicWidget : m_dummyTopicWidgets) {
        if (dummyTopicWidget->getTopicName().isEmpty()) {
            Utils::UI::createCriticalMessageBox("Empty topic name!", "Please enter a topic name for every topic!");
            return;
        }

        if (m_checkROS2NameConform && !Utils::ROS::isNameROS2Conform(dummyTopicWidget->getTopicName()) && areROS2NamesValid) {
            if (const auto returnValue = Utils::UI::continueWithInvalidROS2Names(); !returnValue) {
                return;
            }
            // Only ask once for invalid names
            areROS2NamesValid = false;
        }

        topicNameSet.insert(dummyTopicWidget->getTopicName());
    }
    if (topicNameSet.size() != m_dummyTopicWidgets.size()) {
        Utils::UI::createCriticalMessageBox("Duplicate topic names!", "Please make sure that no duplicate topic names are used!");
        return;
    }
    if (!Utils::UI::continueForExistingTarget(m_parameters.sourceDirectory, "Bagfile", "bag file")) {
        return;
    }

    emit okPressed();
}


void
DummyBagWidget::setPixmapLabelIcon()
{
    const auto isDarkMode = Utils::UI::isDarkMode();
    m_headerPixmapLabel->setPixmap(QIcon(isDarkMode ? m_iconPath + "_white.svg" : m_iconPath + "_black.svg").pixmap(QSize(100, 45)));
    m_minusButton->setIcon(QIcon(isDarkMode ? ":/icons/minus_white.svg" : ":/icons/minus_black.svg"));
    m_plusButton->setIcon(QIcon(isDarkMode ? ":/icons/plus_white.svg" : ":/icons/plus_black.svg"));
}


bool
DummyBagWidget::event(QEvent *event)
{
    [[unlikely]] if (event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) {
        setPixmapLabelIcon();
    }
    return QWidget::event(event);
}
