#include "BagTreeWidget.hpp"

#include <QLabel>
#include <QTreeWidgetItem>

BagTreeWidget::BagTreeWidget(QWidget *parent) : QTreeWidget(parent)
{
    setColumnCount(3);
    setMinimumHeight(MINIMUM_HEIGHT);
    setMaximumHeight(MAXIMUM_HEIGHT);

    headerItem()->setText(COL_CHECKBOXES, "");
    headerItem()->setText(COL_TOPIC_NAME, "Topic Name:");
    headerItem()->setText(COL_TOPIC_TYPE, "Topic Type:");
    setRootIsDecorated(false);
    setVisible(false);

    connect(this, &QTreeWidget::itemChanged, this, &BagTreeWidget::itemCheckStateChanged);
}


void
BagTreeWidget::createItemWithTopicNameAndType(const QString& topicName, const QString& topicType,
                                              bool isSelected, QTreeWidgetItem* parentItem)
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
BagTreeWidget::setTreeWidgetItemSelection(Qt::CheckState checkState)
{
    // Type has to be specified to avoid clang compiler errors
    const auto selectItemAndChildren = [checkState] (auto&& selectItemAndChildren, QTreeWidgetItem* item) -> void {
        // Merge bags widget tree has top items without checkboxes
        if (!item->data(COL_CHECKBOXES, Qt::CheckStateRole).isNull()) {
            item->setCheckState(COL_CHECKBOXES, checkState);
        }
        // Apply recursively to all child items as well
        for (auto i = 0; i < item->childCount(); ++i) {
            selectItemAndChildren(selectItemAndChildren, item->child(i));
        }
    };

    for (auto i = 0; i < topLevelItemCount(); ++i) {
        selectItemAndChildren(selectItemAndChildren, topLevelItem(i));
    }
}


std::pair<int, int>
BagTreeWidget::countSelectedAndTotalItems() const
{
    // Type has to be specified to avoid clang compiler errors
    const auto countItemAndChildren = [] (auto&& countItemAndChildren, const QTreeWidgetItem* item) -> std::pair<int, int> {
        auto selectedCount = 0;
        auto checkBoxCount = 0;
        // Merge bags widget tree has top items without checkboxes
        if (!item->data(COL_CHECKBOXES, Qt::CheckStateRole).isNull()) {
            checkBoxCount++;
            selectedCount += item->checkState(COL_CHECKBOXES) == Qt::Checked;
        }
        // Count child items as well
        for (auto i = 0; i < item->childCount(); ++i) {
            const auto [childSelectedCount, childCheckBoxCount] = countItemAndChildren(countItemAndChildren, item->child(i));
            selectedCount += childSelectedCount;
            checkBoxCount += childCheckBoxCount;
        }
        return { selectedCount, checkBoxCount };
    };

    auto selectedCount = 0;
    auto checkBoxCount = 0;

    for (auto i = 0; i < topLevelItemCount(); ++i) {
        const auto [selected, total] = countItemAndChildren(countItemAndChildren, topLevelItem(i));
        selectedCount += selected;
        checkBoxCount += total;
    }

    return { selectedCount, checkBoxCount };
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
