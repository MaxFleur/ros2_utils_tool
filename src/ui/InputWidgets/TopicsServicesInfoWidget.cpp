#include "TopicsServicesInfoWidget.hpp"

#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "UtilsROS.hpp"

TopicsServicesInfoWidget::TopicsServicesInfoWidget(QWidget *parent) :
    BasicInputWidget("Topics and\nServices Info", ":/icons/topics_services_info", parent)
{
    m_treeWidget = new QTreeWidget;
    m_treeWidget->setColumnCount(2);
    m_treeWidget->headerItem()->setText(COL_NAME, "Name");
    m_treeWidget->headerItem()->setText(COL_TYPE, "Type");
    m_treeWidget->setRootIsDecorated(false);
    m_treeWidget->setMinimumWidth(550);
    m_treeWidget->setMinimumHeight(300);

    fillTree();

    auto* const controlsLayout = new QVBoxLayout;
    controlsLayout->addStretch();
    controlsLayout->addWidget(m_headerPixmapLabel);
    controlsLayout->addWidget(m_headerLabel);
    controlsLayout->addSpacing(30);
    controlsLayout->addWidget(m_treeWidget);
    controlsLayout->addStretch();
    // Give it a more "squishy" look
    controlsLayout->setContentsMargins(30, 30, 30, 30);
    controlsLayout->addStretch();

    auto* const controlsSqueezedLayout = new QHBoxLayout;
    controlsSqueezedLayout->addStretch();
    controlsSqueezedLayout->addLayout(controlsLayout);
    controlsSqueezedLayout->addStretch();

    auto* const refreshButton = new QPushButton("Refresh");
    m_dialogButtonBox->addButton(refreshButton, QDialogButtonBox::AcceptRole);
    m_okButton->setVisible(false);

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(controlsSqueezedLayout);
    mainLayout->addLayout(m_buttonLayout);
    setLayout(mainLayout);

    connect(refreshButton, &QPushButton::clicked, this, &TopicsServicesInfoWidget::fillTree);
}


void
TopicsServicesInfoWidget::fillTree() const
{
    m_treeWidget->clear();

    // Fill tree with info data
    QList<QTreeWidgetItem*> treeWidgetItems;

    const auto& topicsData = Utils::ROS::getTopicInformation();
    auto* const topicsItem = new QTreeWidgetItem({ "Current Topics:" });

    for (const auto& entry : topicsData) {
        auto* const topicNameItem = new QTreeWidgetItem(topicsItem);
        topicNameItem->setText(COL_NAME, QString::fromStdString(entry.first));
        topicNameItem->setText(COL_TYPE, QString::fromStdString(entry.second.at(0)));

        auto* const publisherCountItem = new QTreeWidgetItem(topicNameItem);
        publisherCountItem->setText(COL_NAME, "Number of Publishers:");
        publisherCountItem->setText(COL_TYPE, QString::fromStdString(entry.second.at(1)));
        auto* const subscriberCountItem = new QTreeWidgetItem(topicNameItem);
        subscriberCountItem->setText(COL_NAME, "Number of Subscribers:");
        subscriberCountItem->setText(COL_TYPE, QString::fromStdString(entry.second.at(2)));
    }

    treeWidgetItems.append(topicsItem);
    treeWidgetItems.append(new QTreeWidgetItem({ "", "" }));

    const auto& servicesData = Utils::ROS::getServiceNamesAndTypes();
    auto* const servicesItem = new QTreeWidgetItem({ "Current Services:" });

    for (const auto& entry : servicesData) {
        auto* const serviceItem = new QTreeWidgetItem(servicesItem);
        serviceItem->setText(COL_NAME, QString::fromStdString(entry.first));
        serviceItem->setText(COL_TYPE, QString::fromStdString(entry.second.at(0)));
    }

    treeWidgetItems.append(servicesItem);

    auto font = topicsItem->font(COL_NAME);
    font.setBold(true);
    topicsItem->setFont(COL_NAME, font);
    servicesItem->setFont(COL_NAME, font);

    m_treeWidget->addTopLevelItems(treeWidgetItems);
    m_treeWidget->expandAll();
    m_treeWidget->resizeColumnToContents(COL_NAME);
}
