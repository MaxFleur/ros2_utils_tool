#include "BagTreeWidget.hpp"

#include <QLabel>

BagTreeWidget::BagTreeWidget(QWidget *parent) : QTreeWidget(parent)
{
    setColumnCount(3);
    headerItem()->setText(COL_CHECKBOXES, "");
    headerItem()->setText(COL_TOPIC_NAME, "Topic Name:");
    headerItem()->setText(COL_TOPIC_TYPE, "Topic Type:");
    setRootIsDecorated(false);
    setVisible(false);

    connect(this, &QTreeWidget::itemChanged, this, &BagTreeWidget::itemCheckStateChanged);
}


void
BagTreeWidget::createItemWithTopicNameAndType(const QString& topicName, const QString& topicType, bool isSelected, QTreeWidgetItem* parentItem)
{
    auto* const item = new QTreeWidgetItem;
    parentItem ? parentItem->addChild(item) : addTopLevelItem(item);

    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
    item->setCheckState(COL_CHECKBOXES, isSelected ? Qt::Checked : Qt::Unchecked);

    auto* const topicNameLabel = new QLabel(topicName);
    topicNameLabel->setEnabled(isSelected);

    auto* const topicTypeLabel = new QLabel(topicType);
    topicTypeLabel->setEnabled(isSelected);
    auto font = topicTypeLabel->font();
    font.setItalic(true);
    topicTypeLabel->setFont(font);

    setItemWidget(item, COL_TOPIC_NAME, topicNameLabel);
    setItemWidget(item, COL_TOPIC_TYPE, topicTypeLabel);
}


void
BagTreeWidget::resizeColumns()
{
    for (auto i = 0; i < columnCount(); ++i) {
        resizeColumnToContents(i);
    }
}


void
BagTreeWidget::itemCheckStateChanged(QTreeWidgetItem* item, int column)
{
    if (column != COL_CHECKBOXES) {
        return;
    }

    // Disable item widgets, this improves distinction between enabed and disabled topics
    itemWidget(item, COL_TOPIC_NAME)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);
    itemWidget(item, COL_TOPIC_TYPE)->setEnabled(item->checkState(COL_CHECKBOXES) == Qt::Checked ? true : false);
}
