#include "TopicsServicesInfoWidget.hpp"

#include "LoadingWidget.hpp"
#include "TopicsServicesThread.hpp"
#include "UtilsROS.hpp"

#include <QDialogButtonBox>
#include <QLabel>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

TopicsServicesInfoWidget::TopicsServicesInfoWidget(QWidget *parent) :
    BasicInputWidget("Topics and\nServices Info", ":/icons/tools/topics_services_info", parent)
{
    m_treeWidget = new QTreeWidget;
    m_treeWidget->setColumnCount(2);
    m_treeWidget->headerItem()->setText(COL_NAME, "Name");
    m_treeWidget->headerItem()->setText(COL_TYPE, "Type");
    m_treeWidget->setRootIsDecorated(false);
    m_treeWidget->setMinimumWidth(550);
    m_treeWidget->setMinimumHeight(300);
    m_treeWidget->setVisible(false);

    m_loadingWidget = new LoadingWidget;

    m_controlsLayout->addStretch();
    m_controlsLayout->addWidget(m_headerPixmapLabel);
    m_controlsLayout->addWidget(m_headerLabel);
    m_controlsLayout->addSpacing(30);
    m_controlsLayout->addWidget(m_treeWidget);
    m_controlsLayout->addWidget(m_loadingWidget);
    m_controlsLayout->addStretch();

    auto* const refreshButton = new QPushButton("Refresh");
    m_dialogButtonBox->addButton(refreshButton, QDialogButtonBox::AcceptRole);
    m_okButton->setVisible(false);

    connect(refreshButton, &QPushButton::clicked, this, &TopicsServicesInfoWidget::startSearchingThread);

    startSearchingThread();
}


void
TopicsServicesInfoWidget::startSearchingThread()
{
    if (m_thread && m_thread->isRunning()) {
        return;
    }

    m_treeWidget->clear();
    m_treeWidget->setVisible(false);
    m_loadingWidget->startLoading();

    m_thread = new TopicsServicesThread;
    connect(m_thread, &QThread::finished, this, &TopicsServicesInfoWidget::handleTreeWidgetPopulation);
    connect(m_thread, &QThread::finished, m_thread, &QObject::deleteLater);
    m_thread->start();
}


void
TopicsServicesInfoWidget::handleTreeWidgetPopulation()
{
    if (!m_thread) {
        return;
    }
    const auto topicsData = m_thread->getTopicInformation();
    const auto servicesData = m_thread->getServiceNamesAndTypes();

    // Fill tree with info data
    QList<QTreeWidgetItem*> treeWidgetItems;

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

    m_loadingWidget->stopLoading();
    m_treeWidget->setVisible(true);
}
