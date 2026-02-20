#include "EditBagWidget.hpp"

#include "BagTreeWidget.hpp"
#include "LowDiskSpaceWidget.hpp"
#include "MessageCountWidget.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <QCheckBox>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>

#include <filesystem>

EditBagWidget::EditBagWidget(Parameters::EditBagParameters& parameters, bool warnROS2NameConvention, QWidget *parent) :
    AdvancedBagWidget(parameters, "Edit Bag", ":/icons/tools/edit_bag", "edit_bag", OUTPUT_BAG_EDITED, parent),
    m_parameters(parameters), m_settings(parameters, "edit_bag"),
    m_warnROS2NameConvention(warnROS2NameConvention)
{
    m_editLabel = new QLabel("Unselect all items you want to remove. Change the message count to crop messages.");
    m_editLabel->setVisible(false);

    m_treeWidget->setColumnCount(4);
    m_treeWidget->headerItem()->setText(COL_MESSAGE_COUNT, "Message Count:");
    m_treeWidget->headerItem()->setText(COL_RENAMING, "Rename Topic (Optional):");

    m_differentDirsLabel = new QLabel("A new bag file will be created, so make sure that enough space is available!");
    m_differentDirsLabel->setVisible(false);

    auto labelFont = m_editLabel->font();
    labelFont.setBold(true);
    m_editLabel->setFont(labelFont);
    m_differentDirsLabel->setFont(labelFont);

    m_deleteSourceCheckBox->setVisible(false);

    m_updateTimestampsCheckBox = new QCheckBox("Update Timestamps to current Time");
    m_updateTimestampsCheckBox->setToolTip("Whether to keep the old bag's timestamp or use the current time<br>when the edited bag is written.");
    m_updateTimestampsCheckBox->setTristate(false);
    m_updateTimestampsCheckBox->setChecked(m_parameters.updateTimestamps);
    m_updateTimestampsCheckBox->setVisible(false);

    m_controlsLayout->addWidget(m_editLabel);
    m_controlsLayout->addWidget(m_treeWidget);
    m_controlsLayout->addWidget(m_findTargetWidget);
    m_controlsLayout->addWidget(m_lowDiskSpaceWidget);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(m_differentDirsLabel);
    m_controlsLayout->addWidget(m_deleteSourceCheckBox);
    m_controlsLayout->addWidget(m_updateTimestampsCheckBox);
    // Give it a more "squishy" look
    m_controlsLayout->setContentsMargins(30, 30, 30, 30);
    m_controlsLayout->addStretch();

    connect(m_updateTimestampsCheckBox, &QCheckBox::stateChanged, this, [this] (int state) {
        writeParameterToSettings(m_parameters.updateTimestamps, state == Qt::Checked, m_settings);
    });

    if (!m_sourceLineEdit->text().isEmpty()) {
        createTopicTree();
    }
}


void
EditBagWidget::findSourceButtonPressed()
{
    const auto bagDirectory = Utils::UI::isBagDirectoryValid(this);
    if (bagDirectory == std::nullopt) {
        return;
    }

    writeParameterToSettings(m_parameters.sourceDirectory, *bagDirectory, m_settings);
    m_parameters.topics.clear();
    m_settings.write();

    m_sourceLineEdit->setText(*bagDirectory);
    fillTargetLineEdit();
    createTopicTree();
}


void
EditBagWidget::createTopicTree()
{
    m_treeWidget->clear();
    m_treeWidget->blockSignals(true);

    const auto& bagMetaData = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
    // Fill tree widget with topics
    for (size_t i = 0; i < bagMetaData.topics_with_message_count.size(); i++) {
        const auto topicWithMessageCount = bagMetaData.topics_with_message_count.at(i);
        const auto& topicMetaData = topicWithMessageCount.topic_metadata;

        const auto it = std::find_if(m_parameters.topics.begin(), m_parameters.topics.end(), [topicMetaData] (const auto& editBagTopic) {
            return editBagTopic.name.toStdString() == topicMetaData.name;
        });
        // If the settings do not contain any topic items, create them
        const auto itemAlreadyExists = it != m_parameters.topics.end();
        if (!itemAlreadyExists) {
            Parameters::EditBagParameters::EditBagTopic editBagTopic;
            editBagTopic.name = QString::fromStdString(topicMetaData.name);
            editBagTopic.upperBoundary = topicWithMessageCount.message_count - 1;
            m_parameters.topics.push_back(editBagTopic);
        }

        auto& editBagTopic = itemAlreadyExists ? *it : m_parameters.topics.back();
        m_treeWidget->createItemWithTopicNameAndType(QString::fromStdString(topicMetaData.name), QString::fromStdString(topicMetaData.type),
                                                     editBagTopic.isSelected);

        auto* const messageCountWidget = new MessageCountWidget(editBagTopic.lowerBoundary, topicWithMessageCount.message_count - 1, editBagTopic.upperBoundary);
        messageCountWidget->setEnabled(editBagTopic.isSelected);

        auto* const renamingLineEdit = new QLineEdit(editBagTopic.renamedName);
        renamingLineEdit->setEnabled(editBagTopic.isSelected);

        auto* lastItem = m_treeWidget->topLevelItem(m_treeWidget->topLevelItemCount() - 1);
        m_treeWidget->setItemWidget(lastItem, COL_MESSAGE_COUNT, messageCountWidget);
        m_treeWidget->setItemWidget(lastItem, COL_RENAMING, renamingLineEdit);

        connect(messageCountWidget, &MessageCountWidget::lowerValueChanged, this, [i, this](int value) {
            writeParameterToSettings(m_parameters.topics[i].lowerBoundary, static_cast<size_t>(value), m_settings);
        });
        connect(messageCountWidget, &MessageCountWidget::upperValueChanged, this, [i, this](int value) {
            writeParameterToSettings(m_parameters.topics[i].upperBoundary, static_cast<size_t>(value), m_settings);
        });
        connect(renamingLineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
            writeParameterToSettings(m_parameters.topics[i].renamedName, text, m_settings);
        });
    }

    m_treeWidget->resizeColumns();
    m_treeWidget->blockSignals(false);
    // Just take a random item to get its height
    auto* item = m_treeWidget->topLevelItem(m_treeWidget->topLevelItemCount() - 1);
    const auto height = m_treeWidget->visualItemRect(item).height();
    m_treeWidget->setFixedHeight((height * m_treeWidget->topLevelItemCount()) + HEIGHT_OFFSET);

    m_editLabel->setVisible(true);
    m_treeWidget->setVisible(true);
    m_differentDirsLabel->setVisible(true);
    m_findTargetWidget->setVisible(true);
    m_deleteSourceCheckBox->setVisible(true);
    m_updateTimestampsCheckBox->setVisible(true);
    m_okButton->setVisible(true);
}


void
EditBagWidget::itemCheckStateChanged(QTreeWidgetItem* item, int column)
{
    if (column != COL_CHECKBOXES) {
        return;
    }

    m_treeWidget->itemWidget(item, COL_MESSAGE_COUNT)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);
    m_treeWidget->itemWidget(item, COL_RENAMING)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);

    const auto rowIndex = m_treeWidget->indexOfTopLevelItem(item);
    writeParameterToSettings(m_parameters.topics[rowIndex].isSelected, item->checkState(COL_CHECKBOXES) == Qt::Checked, m_settings);
}


void
EditBagWidget::okButtonPressed() const
{
    auto areROS2NamesValid = true;
    for (const auto& topic : m_parameters.topics) {
        if (topic.upperBoundary <= topic.lowerBoundary) {
            auto *const msgBox = new QMessageBox(QMessageBox::Critical, "Message count invalid!",
                                                 "Please make sure that the lower message count is actually lower than the higher message count!",
                                                 QMessageBox::Ok);
            msgBox->exec();
            return;
        }
        if (!topic.renamedName.isEmpty() && m_warnROS2NameConvention &&
            !Utils::ROS::isNameROS2Conform(topic.renamedName) && areROS2NamesValid) {
            // Ask only once for invalid names
            areROS2NamesValid = false;
        }
    }

    // Sets remove duplicates, so use such an instance to check for duplicate topic names
    QSet<QString> topicNameSet;
    auto sizeOfSelectedTopics = 0;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        topicNameSet.insert(topic.renamedName.isEmpty() ? topic.name : topic.renamedName);
        sizeOfSelectedTopics++;
    }

    if (const auto ioParamsValid = areIOParametersValid(sizeOfSelectedTopics, topicNameSet.size()); !ioParamsValid) {
        return;
    }
    if (!areROS2NamesValid) {
        if (const auto returnValue = Utils::UI::continueWithInvalidROS2Names(); !returnValue) {
            return;
        }
    }

    emit okPressed();
}
