#include "ConfigurePlayBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "UtilsROS.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <filesystem>

ConfigurePlayBagWidget::ConfigurePlayBagWidget(Parameters::PlayBagParameters& parameters, QWidget *parent) :
    BasicBagWidget(parameters, "Play Bag", ":/icons/tools/play_bag", "play_bag", "Unselect all items you don't want to play.", parent),
    m_parameters(parameters), m_settings(parameters, "play_bag")
{
    if (!std::filesystem::exists(m_parameters.sourceDirectory.toStdString()) || !Utils::ROS::doesDirectoryContainBagFile(m_parameters.sourceDirectory)) {
        m_parameters.sourceDirectory = QString();
        writeParameterToSettings(m_parameters.sourceDirectory, QString(), m_settings);
    }

    m_okButton->setVisible(false);

    auto* const sourceFormLayout = new QFormLayout;
    sourceFormLayout->addRow("Source Bag:", m_findSourceLayout);

    m_lowerOptionsLayout = new QFormLayout;
    m_lowerOptionsLayout->setLabelAlignment(Qt::AlignLeft);

    m_rateSpinBox = new QDoubleSpinBox;
    m_rateSpinBox->setDecimals(NUMBER_OF_DECIMALS);
    m_rateSpinBox->setRange(SPINBOX_LOWER_RANGE, SPINBOX_UPPER_RANGE);
    m_rateSpinBox->setValue(m_parameters.rate);

    m_loopCheckBox = new QCheckBox;
    m_loopCheckBox->setTristate(false);
    m_loopCheckBox->setChecked(m_parameters.loop);

    m_controlsLayout->addSpacing(30);
    m_controlsLayout->addLayout(sourceFormLayout);
    m_controlsLayout->addSpacing(5);
    m_controlsLayout->addWidget(m_unselectLabel);
    m_controlsLayout->addWidget(m_treeWidget);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addLayout(m_lowerOptionsLayout);
    m_controlsLayout->addStretch();

    connect(m_rateSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] (double value) {
        writeParameterToSettings(m_parameters.rate, value, m_settings);
    });
    connect(m_loopCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.loop, state == Qt::Checked, m_settings);
    });

    if (!m_sourceLineEdit->text().isEmpty()) {
        populateTreeWidget();
    }
}


void
ConfigurePlayBagWidget::handleTreeAfterSource()
{
    m_parameters.topics.clear();
    populateTreeWidget();
}


void
ConfigurePlayBagWidget::populateTreeWidget()
{
    m_parameters.services.clear();
    m_parameters.topics.clear();
    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    const auto& bagMetaData = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
    // Fill tree widget with topics
    for (size_t i = 0; i < bagMetaData.topics_with_message_count.size(); i++) {
        const auto topicWithMessageCount = bagMetaData.topics_with_message_count.at(i);
        const auto& topicMetaData = topicWithMessageCount.topic_metadata;

        const QString topicNameQString(QString::fromStdString(topicMetaData.name));
        // Each message always contains the '/msg/' substring. With this, we can differentiate between messages and services.
        auto& vectorToStore = QString::fromStdString(topicMetaData.type).contains("/msg/") ? m_parameters.topics : m_parameters.services;

        const auto it = std::ranges::find_if(vectorToStore, [topicMetaData] (const auto& playBagTopic) {
            return playBagTopic.name.toStdString() == topicMetaData.name;
        });

        // If the settings do not contain any topic items, create them
        const auto itemAlreadyExists = it != vectorToStore.end();
        if (!itemAlreadyExists) {
            vectorToStore.push_back({ { topicNameQString }, true });
        }

        auto& playBagTopic = itemAlreadyExists ? *it : vectorToStore.back();
        m_treeWidget->createItemWithTopicNameAndType(topicNameQString, QString::fromStdString(topicMetaData.type), playBagTopic.isSelected);
    }
    m_settings.write();

    m_treeWidget->resizeColumns();
    m_treeWidget->blockSignals(false);

    m_unselectLabel->setVisible(true);
    m_treeWidget->setVisible(true);
    m_okButton->setVisible(true);

    enableOkButton();

    // Only add the spinbox and checkbox once
    if (m_rowsAdded) {
        return;
    }

    m_lowerOptionsLayout->addRow("Rate:", m_rateSpinBox);
    m_lowerOptionsLayout->addRow("Loop File:", m_loopCheckBox);
    m_rowsAdded = true;
}


void
ConfigurePlayBagWidget::enableOkButton()
{
    const auto isAnyTopicEnabled = std::ranges::any_of(m_parameters.topics, [] (const auto& topic) {
        return topic.isSelected == true;
    });
    const auto isAnyServiceEnabled = std::ranges::any_of(m_parameters.services, [] (const auto& service) {
        return service.isSelected == true;
    });
    m_okButton->setEnabled(isAnyTopicEnabled || isAnyServiceEnabled);
}
