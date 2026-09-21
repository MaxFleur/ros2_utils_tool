#pragma once

#include <QTreeWidget>

#include <utility>

class QTreeWidgetItem;

// Widget for displaying bag contents in a tree
class BagTreeWidget : public QTreeWidget
{
    Q_OBJECT
public:
    explicit
    BagTreeWidget(QWidget* parent = 0);

    void
    createItemWithTopicNameAndType(const QString&   topicName,
                                   const QString&   topicType,
                                   bool             isSelected,
                                   QTreeWidgetItem* parentItem = 0);

    void
    resizeColumns();

    // Returns the number of checked items and the total number of items with checkboxes
    // Works recursively, all child items are included
    [[nodiscard]] std::pair<int, int>
    countSelectedAndTotalItems() const;

public slots:
    void
    setTreeWidgetItemSelection(Qt::CheckState checkState);

private slots:
    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column);

private:
    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;
};
