#include "DummyBagWidget.hpp"

#include "DummyTopicWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSet>
#include <QSpinBox>

DummyBagWidget::DummyBagWidget(Parameters::DummyBagParameters& parameters, bool checkROS2NameConform, QWidget *parent) :
    TopicListingInputWidget(parameters, "Create Dummy Bag", ":/icons/dummy_bag", "dummy_bag", parent),
    m_parameters(parameters), m_settings(parameters, "dummy_bag"),
    m_checkROS2NameConform(checkROS2NameConform)
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
    m_controlsLayout->addSpacing(20);
    m_controlsLayout->addStretch();

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

    connect(messageCountSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        writeParameterToSettings(m_parameters.messageCount, value, m_settings);
    });
    connect(useCustomRateCheckBox, &QCheckBox::stateChanged, this, &DummyBagWidget::useCustomRateCheckBoxPressed);
    connect(m_removeTopicButton, &QPushButton::clicked, this, &DummyBagWidget::removeDummyTopicWidget);
    connect(m_addTopicButton, &QPushButton::clicked, this, [addNewTopic] {
        addNewTopic();
    });

    setPixmapLabelIcon();
    useCustomRateCheckBoxPressed(m_parameters.useCustomRate);
}


void
DummyBagWidget::removeDummyTopicWidget()
{
    m_formLayout->removeRow(m_parameters.topics.size());
    m_dummyTopicWidgets.pop_back();
    m_parameters.topics.pop_back();
    m_settings.write();
    m_numberOfTopics--;

    m_addTopicButton->setEnabled(m_numberOfTopics != MAXIMUM_NUMBER_OF_TOPICS);
    m_removeTopicButton->setEnabled(m_parameters.topics.size() != 1);
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
    // Ensure that the plus and minus button stay below the newly formed widget (row count 4 means no topic widgets yet)
    m_formLayout->insertRow(m_formLayout->rowCount() == 4 ? m_formLayout->rowCount() - 3 : m_formLayout->rowCount() - 4,
                            "Topic " + QString::number(m_numberOfTopics + 1) + ":", dummyTopicWidget);
    m_dummyTopicWidgets.push_back(dummyTopicWidget);

    m_numberOfTopics++;
    m_addTopicButton->setEnabled(m_numberOfTopics != MAXIMUM_NUMBER_OF_TOPICS);
    m_removeTopicButton->setEnabled(m_parameters.topics.size() != 1);
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

    for (QPointer<DummyTopicWidget> dummyTopicWidget : m_dummyTopicWidgets) {
        if (dummyTopicWidget->getTopicName().isEmpty()) {
            Utils::UI::createCriticalMessageBox("Empty topic name!", "Please enter a topic name for every topic!");
            return std::nullopt;
        }
        if (m_checkROS2NameConform && !Utils::ROS::isNameROS2Conform(dummyTopicWidget->getTopicName()) && areROS2NamesValid) {
            if (const auto returnValue = Utils::UI::continueWithInvalidROS2Names(); !returnValue) {
                return std::nullopt;
            }
            // Only ask once for invalid names
            areROS2NamesValid = false;
        }

        topicNameSet.insert(dummyTopicWidget->getTopicName());
    }

    return topicNameSet.size() == m_dummyTopicWidgets.size();
}
