#include "BagInfoWidget.hpp"

#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"

#include <chrono>

BagInfoWidget::BagInfoWidget(QWidget *parent) :
    BasicInputWidget("Bag Info", ":/icons/tools/bag_info", parent)
{
    // Don't need it here
    m_dialogButtonBox->setVisible(false);

    auto* const bagLineEdit = new QLineEdit;
    bagLineEdit->setToolTip("The source bag file directory.");

    auto* const formLayout = new QFormLayout;
    formLayout->addRow("Bag File:", m_findSourceLayout);

    m_infoTreeWidget = new QTreeWidget;
    m_infoTreeWidget->setColumnCount(2);
    m_infoTreeWidget->headerItem()->setText(COL_DESCRIPTION, "Metadata");
    m_infoTreeWidget->headerItem()->setText(COL_INFORMATION, "Values");
    m_infoTreeWidget->setVisible(false);
    m_infoTreeWidget->setRootIsDecorated(false);
    m_infoTreeWidget->setMinimumWidth(430);
    m_infoTreeWidget->setMinimumHeight(300);

    m_controlsLayout->addSpacing(30);
    m_controlsLayout->addLayout(formLayout);
    m_controlsLayout->addSpacing(10);
    m_controlsLayout->addWidget(m_infoTreeWidget);
    m_controlsLayout->addStretch();

    connect(m_findSourceButton, &QPushButton::clicked, this, &BagInfoWidget::displayBagInfo);
    connect(m_okButton, &QPushButton::clicked, this, [this] {
        emit back();
    });
}


void
BagInfoWidget::displayBagInfo()
{
    const auto bagDirectory = QFileDialog::getExistingDirectory(this, "Open Source Bag File", "",
                                                                QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (bagDirectory.isEmpty()) {
        return;
    }
    if (!Utils::ROS::doesDirectoryContainBagFile(bagDirectory)) {
        auto *const msgBox = new QMessageBox(QMessageBox::Critical, "No ROS bag detected!",
                                             "The specified directory contains no ROS bag file!", QMessageBox::Ok);
        msgBox->exec();
        return;
    }

    m_infoTreeWidget->clear();
    m_sourceLineEdit->setText(bagDirectory);
    const auto& bagMetaData = Utils::ROS::getBagMetadata(bagDirectory);
    // Fill tree with bag data
    QList<QTreeWidgetItem*> treeWidgetItems;
    treeWidgetItems.append(new QTreeWidgetItem({ "Duration (Nanoseconds):", QString::number(bagMetaData.duration.count()) }));
    const auto durationInSeconds = QString::number(static_cast<float>(bagMetaData.duration.count() / 1e9), 'f', 3);
    treeWidgetItems.append(new QTreeWidgetItem({ "Duration (Seconds):", durationInSeconds == "0.000" ? "0.001" : durationInSeconds }));

    const auto timeValue = std::chrono::system_clock::to_time_t(bagMetaData.starting_time);
    std::stringstream stringStream;
    stringStream << std::put_time(std::localtime(&timeValue), "%F %T");
    // Use a more understandable time format
    treeWidgetItems.append(new QTreeWidgetItem({ "Starting Time:", QString::fromStdString(stringStream.str()) }));
    // Only show the last three digits
    treeWidgetItems.append(new QTreeWidgetItem({ "Size:",
                                                 QString::number(static_cast<float>(bagMetaData.bag_size /
                                                                                    static_cast<float>(Utils::General::GIGABYTE_IN_BYTES)), 'f', 3) + " GiB" }));
    treeWidgetItems.append(new QTreeWidgetItem({ "Size (Message Count):", QString::number(bagMetaData.message_count) }));
    treeWidgetItems.append(new QTreeWidgetItem({ "Storage Identifier:", QString::fromStdString(bagMetaData.storage_identifier) }));
    treeWidgetItems.append(new QTreeWidgetItem({ "Compression Format:",
                                                 bagMetaData.compression_mode == "" ? "None": QString::fromStdString(bagMetaData.compression_mode) }));
    treeWidgetItems.append(new QTreeWidgetItem({ "Compression Mode:",
                                                 bagMetaData.compression_format == "" ? "None": QString::fromStdString(bagMetaData.compression_format) }));
    if (!bagMetaData.ros_distro.empty()) {
        treeWidgetItems.append(new QTreeWidgetItem({ "ROS Distro:", QString::fromStdString(bagMetaData.ros_distro) }));
    }
    treeWidgetItems.append(new QTreeWidgetItem({ "Version:", QString::number(bagMetaData.version) }));
    // Some empty space before the topic messages
    treeWidgetItems.append(new QTreeWidgetItem({ "", "" }));

    auto* const topicMetaDataItem = new QTreeWidgetItem({ "Topic Information:" });

    for (const auto& topicInformation : bagMetaData.topics_with_message_count) {
        const auto& topicMetaData = topicInformation.topic_metadata;

        auto* const topicInformationItem = new QTreeWidgetItem(topicMetaDataItem);
        topicInformationItem->setText(COL_DESCRIPTION, QString::fromStdString(topicMetaData.name));

        auto* const topicTypeItem = new QTreeWidgetItem(topicInformationItem);
        topicTypeItem->setText(COL_DESCRIPTION, "Type:");
        topicTypeItem->setText(COL_INFORMATION, QString::fromStdString(topicMetaData.type));
        auto* const topicMessageCountItem = new QTreeWidgetItem(topicInformationItem);
        topicMessageCountItem->setText(COL_DESCRIPTION, "Message Count:");
        topicMessageCountItem->setText(COL_INFORMATION, QString::number(topicInformation.message_count));
    }

    treeWidgetItems.append(topicMetaDataItem);

    m_infoTreeWidget->addTopLevelItems(treeWidgetItems);
    m_infoTreeWidget->expandAll();
    m_infoTreeWidget->resizeColumnToContents(COL_DESCRIPTION);
    m_infoTreeWidget->setVisible(true);
}
