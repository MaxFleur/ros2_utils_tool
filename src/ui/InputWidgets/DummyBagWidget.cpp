#include "DummyBagWidget.hpp"

#include "LowDiskSpaceWidget.hpp"
#include "TopicWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSet>
#include <QSpinBox>

DummyBagWidget::DummyBagWidget(Parameters::DummyBagParameters& parameters, bool warnROS2NameConvention, QWidget *parent) :
    TopicListingInputWidget(parameters, "Create Dummy Bag", ":/icons/dummy_bag", "dummy_bag", parent),
    m_parameters(parameters), m_settings(parameters, "dummy_bag"),
    m_warnROS2NameConvention(warnROS2NameConvention)
{
    m_sourceLineEdit->setText(parameters.sourceDirectory);
    m_sourceLineEdit->setToolTip("The target dummy bag file directory.");

    auto* const messageCountSpinBox = new QSpinBox;
    messageCountSpinBox->setRange(1, 1000);
    messageCountSpinBox->setToolTip("The number of messages stored in the bag.");
    messageCountSpinBox->setValue(m_parameters.messageCount);

    auto* const useCustomRateCheckBox = Utils::UI::createCheckBox("Use a custom rate for the bag messages. If this is unchecked, "
                                                                  "the current time will be used.", m_parameters.useCustomRate);
    useCustomRateCheckBox->setChecked(m_parameters.useCustomRate);

    auto* const rateSpinBox = new QSpinBox;
    rateSpinBox->setRange(1, 100);
    rateSpinBox->setToolTip("How many messages per second are stored.");
    rateSpinBox->setValue(m_parameters.rate);

    m_formLayout = new QFormLayout;
    m_formLayout->addRow("Target Bag Location:", m_findSourceLayout);
    m_formLayout->addRow("", m_topicButtonLayout);
    m_formLayout->addRow("Message Count:", messageCountSpinBox);
    m_formLayout->addRow("Use Custom Rate:", useCustomRateCheckBox);

    m_controlsLayout->addLayout(m_formLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(m_lowDiskSpaceWidget);
    m_controlsLayout->addSpacing(20);
    m_controlsLayout->addStretch();
    m_controlsLayout->setAlignment(m_lowDiskSpaceWidget, Qt::AlignCenter);

    const auto addNewTopic = [this] {
        m_parameters.topics.push_back({ "String", "" });
        m_settings.write();
        createNewDummyTopicWidget({ { "" }, "" }, m_parameters.topics.size() - 1, false);
    };
    // Create widgets for already existing topics
    for (auto i = 0; i < m_parameters.topics.size(); i++) {
        createNewDummyTopicWidget(m_parameters.topics.at(i), i, true);
    }
    if (m_parameters.topics.empty()) {
        addNewTopic();
    }

    connect(messageCountSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        writeParameterToSettings(m_parameters.messageCount, value, m_settings);
    });
    connect(useCustomRateCheckBox, &QCheckBox::stateChanged, this, &DummyBagWidget::useCustomRateCheckBoxPressed);
    connect(m_addTopicButton, &QPushButton::clicked, this, [addNewTopic] {
        addNewTopic();
    });

    setPixmapLabelIcon();
    setLowDiskSpaceWidgetVisibility(m_sourceLineEdit->text());
    useCustomRateCheckBoxPressed(m_parameters.useCustomRate);
    m_okButton->setEnabled(!m_sourceLineEdit->text().isEmpty());
}


void
DummyBagWidget::removeDummyTopicWidget(int row)
{
    m_formLayout->removeRow(row);
    m_topicLabels.remove(row - 1);
    m_topicWidgets.remove(row - 1);
    m_parameters.topics.remove(row - 1);
    m_settings.write();
    m_numberOfTopics--;

    for (auto i = 0; i < m_numberOfTopics; i++) {
        m_topicWidgets[i]->setProperty("id", i + 1);
    }
    for (auto i = 0; i < m_topicLabels.size(); ++i) {
        m_topicLabels[i]->setText("Topic " + QString::number(i + 1) + ":");
    }

    m_addTopicButton->setEnabled(m_numberOfTopics != MAXIMUM_NUMBER_OF_TOPICS);
}


void
DummyBagWidget::createNewDummyTopicWidget(const Parameters::DummyBagParameters::DummyBagTopic& topic, int index, bool isCtor)
{
    m_topicLabels.push_back(new QLabel("Topic " + QString::number(m_numberOfTopics + 1) + ":"));

    auto* const topicWidget = new TopicWidget(true, m_numberOfTopics != 0, topic.type, topic.name);
    m_topicWidgets.push_back(topicWidget);
    // Keep it all inside the main form layout
    // Ensure that the plus button stays below the newly formed widget
    m_formLayout->insertRow(isCtor || !m_parameters.useCustomRate ? m_formLayout->rowCount() - TOPIC_WIDGET_OFFSET
                                                                  : m_formLayout->rowCount() - TOPIC_WIDGET_OFFSET_WITH_CUSTOM_RATE,
                            m_topicLabels.back(), topicWidget);

    connect(topicWidget, &TopicWidget::topicTypeChanged, this, [this, index] (const QString& text) {
        writeParameterToSettings(m_parameters.topics[index].type, text, m_settings);
    });
    connect(topicWidget, &TopicWidget::topicNameChanged, this, [this, index] (const QString& text) {
        writeParameterToSettings(m_parameters.topics[index].name, text, m_settings);
    });
    connect(topicWidget, &TopicWidget::topicRemoveButtonClicked, this, [this, topicWidget] {
        removeDummyTopicWidget(topicWidget->property("id").toInt());
    });

    m_numberOfTopics++;
    for (auto i = 0; i < m_numberOfTopics; i++) {
        m_topicWidgets[i]->setProperty("id", i + 1);
    }
    m_addTopicButton->setEnabled(m_numberOfTopics != MAXIMUM_NUMBER_OF_TOPICS);
}


void
DummyBagWidget::useCustomRateCheckBoxPressed(int state)
{
    writeParameterToSettings(m_parameters.useCustomRate, state != Qt::Unchecked, m_settings);

    if (state != Qt::Unchecked) {
        m_rateSpinBox = new QSpinBox;
        m_rateSpinBox->setRange(1, 100);
        m_rateSpinBox->setValue(m_parameters.rate);

        m_formLayout->addRow("", m_rateSpinBox);

        connect(m_rateSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
            writeParameterToSettings(m_parameters.rate, value, m_settings);
        });
    } else if (m_rateSpinBox) {
        m_formLayout->removeRow(m_rateSpinBox);
    }
}


std::optional<bool>
DummyBagWidget::areTopicsValid() const
{
    auto areROS2NamesValid = true;
    QSet<QString> topicNameSet;

    for (auto dummyTopicWidget : m_topicWidgets) {
        if (dummyTopicWidget->getTopicName().isEmpty()) {
            Utils::UI::createCriticalMessageBox("Empty topic name!", "Please enter a topic name for every topic!");
            return std::nullopt;
        }
        if (m_warnROS2NameConvention && !Utils::ROS::isNameROS2Conform(dummyTopicWidget->getTopicName()) && areROS2NamesValid) {
            if (const auto returnValue = Utils::UI::continueWithInvalidROS2Names(); !returnValue) {
                return std::nullopt;
            }
            // Only ask once for invalid names
            areROS2NamesValid = false;
        }

        topicNameSet.insert(dummyTopicWidget->getTopicName());
    }

    return topicNameSet.size() == m_topicWidgets.size();
}
