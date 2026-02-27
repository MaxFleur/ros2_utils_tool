#include "MergeBagsWidget.hpp"

#include "BagTreeWidget.hpp"
#include "LowDiskSpaceWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>

#include <filesystem>

MergeBagsWidget::MergeBagsWidget(Parameters::MergeBagsParameters& parameters, QWidget *parent) :
    AdvancedBagWidget(parameters, "Merge Bags", ":/icons/tools/merge_bags", "merge_bags", OUTPUT_BAG_MERGED, parent),
    m_parameters(parameters), m_settings(parameters, "merge_bags")
{
    m_secondSourceLineEdit = new QLineEdit;
    auto* const secondSourceButton = new QToolButton;
    auto* const secondSourceLayout = Utils::UI::createLineEditButtonLayout(m_secondSourceLineEdit, secondSourceButton);
    m_basicOptionsFormLayout->addRow("Second Source Bag:", secondSourceLayout);

    if (!std::filesystem::exists(m_parameters.secondSourceDirectory.toStdString()) ||
        !Utils::ROS::doesDirectoryContainBagFile(m_parameters.secondSourceDirectory)) {
        m_parameters.secondSourceDirectory = "";
        writeParameterToSettings(m_parameters.secondSourceDirectory, QString(), m_settings);
    }
    m_secondSourceLineEdit->setText(m_parameters.secondSourceDirectory);

    m_treeWidget->setMinimumHeight(300);

    m_sufficientSpaceLabel = new QLabel("A new bag file will be created, so make sure that enough space is available!");
    m_sufficientSpaceLabel->setVisible(false);

    auto labelFont = m_sufficientSpaceLabel->font();
    labelFont.setBold(true);
    m_sufficientSpaceLabel->setFont(labelFont);

    m_deleteSourceCheckBox->setVisible(false);

    m_controlsLayout->addWidget(m_treeWidget);
    m_controlsLayout->addWidget(m_findTargetWidget);
    m_controlsLayout->addWidget(m_lowDiskSpaceWidget);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(m_sufficientSpaceLabel);
    m_controlsLayout->addWidget(m_deleteSourceCheckBox);
    // Give it a more "squishy" look
    m_controlsLayout->setContentsMargins(30, 30, 30, 30);
    m_controlsLayout->addStretch();

    connect(secondSourceButton, &QPushButton::clicked, this, [this] {
        m_secondSourceButtonClicked = true;
        findSourceButtonPressed();
        m_secondSourceButtonClicked = false;
    });

    if (!m_sourceLineEdit->text().isEmpty() && !m_secondSourceLineEdit->text().isEmpty()) {
        createTopicTree(false);
    }
}


void
MergeBagsWidget::findSourceButtonPressed()
{
    QPointer<QLineEdit> lineEdit = m_secondSourceButtonClicked ? m_secondSourceLineEdit : m_sourceLineEdit;
    const auto bagDirectory = Utils::UI::isBagDirectoryValid(this);
    if (bagDirectory == std::nullopt) {
        return;
    }
    const auto& otherSourcePath = m_secondSourceButtonClicked ? m_parameters.sourceDirectory : m_parameters.secondSourceDirectory;
    if (bagDirectory == otherSourcePath) {
        Utils::UI::createCriticalMessageBox("Equal input bag files!", "The input files are identical. Please select a different file!");
        return;
    }

    lineEdit->setText(*bagDirectory);
    writeParameterToSettings(m_secondSourceButtonClicked ? m_parameters.secondSourceDirectory : m_parameters.sourceDirectory, *bagDirectory, m_settings);
    m_settings.write();

    if (!m_parameters.sourceDirectory.isEmpty() && !m_parameters.secondSourceDirectory.isEmpty()) {
        fillTargetLineEdit();
        createTopicTree(true);
    }
}


void
MergeBagsWidget::createTopicTree(bool resetTopicsParameter)
{
    if (resetTopicsParameter) {
        m_parameters.topics.clear();
        m_settings.write();
    }

    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    auto topicIndex = 0;
    // Fill tree widget with topics
    const auto fillTreeWithBagTopics = [this, &topicIndex] (const auto& bagFilePath, const auto& topLevelItemText) {
        auto* const topLevelItem = new QTreeWidgetItem;
        topLevelItem->setText(COL_CHECKBOXES, topLevelItemText);

        auto font = topLevelItem->font(COL_CHECKBOXES);
        font.setBold(true);
        topLevelItem->setFont(COL_CHECKBOXES, font);
        m_treeWidget->addTopLevelItem(topLevelItem);

        const auto& bagMetaData = Utils::ROS::getBagMetadata(bagFilePath);

        for (size_t i = 0; i < bagMetaData.topics_with_message_count.size(); i++) {
            const auto topicWithMessageCount = bagMetaData.topics_with_message_count.at(i);
            const auto& topicMetaData = topicWithMessageCount.topic_metadata;

            // Search if the topic already exists in parameters. If it is not there, add it to parameters.
            const auto it = std::ranges::find_if(m_parameters.topics, [topicMetaData, bagFilePath] (const auto& topic) {
                return topic.name.toStdString() == topicMetaData.name && topic.bagDir == bagFilePath;
            });

            const auto itemAlreadyExists = it != m_parameters.topics.end();
            if (!itemAlreadyExists) {
                Parameters::MergeBagsParameters::MergeBagTopic mergeBagTopic;
                mergeBagTopic.name = QString::fromStdString(topicMetaData.name);
                mergeBagTopic.bagDir = bagFilePath;
                m_parameters.topics.push_back(mergeBagTopic);
            }

            auto& mergeBagTopic = itemAlreadyExists ? *it : m_parameters.topics.back();
            m_treeWidget->createItemWithTopicNameAndType(QString::fromStdString(topicMetaData.name), QString::fromStdString(topicMetaData.type),
                                                         mergeBagTopic.isSelected, topLevelItem);
            m_treeWidget->topLevelItem(m_treeWidget->topLevelItemCount() - 1)->setData(COL_TOPIC_NAME, Qt::UserRole, QVariant::fromValue(i + topicIndex));
        }

        topicIndex += bagMetaData.topics_with_message_count.size();
    };

    fillTreeWithBagTopics(m_parameters.sourceDirectory, "First:");
    m_treeWidget->addTopLevelItem(new QTreeWidgetItem);
    fillTreeWithBagTopics(m_parameters.secondSourceDirectory, "Second:");
    m_treeWidget->expandAll();

    m_treeWidget->resizeColumns();
    m_treeWidget->setColumnWidth(COL_TOPIC_NAME, m_treeWidget->columnWidth(COL_TOPIC_NAME) + 10);

    m_treeWidget->blockSignals(false);

    m_treeWidget->setVisible(true);
    m_findTargetWidget->setVisible(true);
    m_sufficientSpaceLabel->setVisible(true);
    m_deleteSourceCheckBox->setVisible(true);
    m_okButton->setVisible(true);
}


void
MergeBagsWidget::itemCheckStateChanged(QTreeWidgetItem* item, int column)
{
    if (column != COL_CHECKBOXES) {
        return;
    }

    // Find corresponding row index in parameters
    auto rowIndex = 0;
    const auto bagDir = item->parent()->text(COL_CHECKBOXES) == "First:" ? m_parameters.sourceDirectory : m_parameters.secondSourceDirectory;

    for (auto i = 0; i < m_parameters.topics.size(); i++) {
        auto* const nameLabel = static_cast<QLabel*>(m_treeWidget->itemWidget(item, COL_TOPIC_NAME));
        // Equal name and bag dir required to find the correct topic
        if (m_parameters.topics.at(i).name == nameLabel->text() && m_parameters.topics.at(i).bagDir == bagDir) {
            rowIndex = i;
            break;
        }
    }
    writeParameterToSettings(m_parameters.topics[rowIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_settings);
}


void
MergeBagsWidget::okButtonPressed() const
{
    // Sets remove duplicates, so use such an instance to check for duplicate topic names
    QSet<QString> topicNameSet;
    auto sizeOfSelectedTopics = 0;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        topicNameSet.insert(topic.name);
        sizeOfSelectedTopics++;
    }

    if (const auto ioParamsValid = areIOParametersValid(sizeOfSelectedTopics, topicNameSet.size(), m_parameters.secondSourceDirectory);
        !ioParamsValid) {
        return;
    }

    emit okPressed();
}
