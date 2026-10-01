#pragma once

#include "BasicInputWidget.hpp"

#include <QPointer>
#include <QWidget>

class LoadingWidget;
class TopicsServicesThread;
class QTreeWidget;

// The widget showing topics and services info data
class TopicsServicesInfoWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    TopicsServicesInfoWidget(QWidget* parent = 0);

private:
    void
    startSearchingThread();

    void
    handleTreeWidgetPopulation();

private:
    QPointer<QTreeWidget> m_treeWidget;

    QPointer<LoadingWidget> m_loadingWidget;

    QPointer<TopicsServicesThread> m_thread;

    static constexpr int COL_NAME = 0;
    static constexpr int COL_TYPE = 1;
};
