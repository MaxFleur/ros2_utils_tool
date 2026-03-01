#pragma once

#include "BasicInputWidget.hpp"

#include <QPointer>
#include <QWidget>

class QTreeWidget;

// The widget showing bag info data
class BagInfoWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    BagInfoWidget(QWidget* parent = 0);

private slots:
    void
    populateTreeWidget();

private:
    // Main tree used to display bag info
    QPointer<QTreeWidget> m_treeWidget;

    static constexpr int COL_DESCRIPTION = 0;
    static constexpr int COL_INFORMATION = 1;
};
