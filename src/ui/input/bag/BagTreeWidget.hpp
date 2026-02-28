#pragma once

#include <QTreeWidget>
#include <QTreeWidgetItem>

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

private slots:
    void
    itemCheckStateChanged(QTreeWidgetItem* item,
                          int              column);

private:
    static constexpr int COL_CHECKBOXES = 0;
    static constexpr int COL_TOPIC_NAME = 1;
    static constexpr int COL_TOPIC_TYPE = 2;
};
