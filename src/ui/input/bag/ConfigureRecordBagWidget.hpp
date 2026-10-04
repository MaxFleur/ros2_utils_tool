#pragma once

#include "BasicBagWidget.hpp"
#include "Parameters.hpp"
#include "RecordBagSettings.hpp"

#include <QPointer>
#include <QWidget>

class BagTreeWidget;
class LoadingWidget;
class TopicsServicesThread;

class QPushButton;

// Widget used to manage recording a bag file
class ConfigureRecordBagWidget : public BasicBagWidget
{
    Q_OBJECT

public:
    ConfigureRecordBagWidget(Parameters::RecordBagParameters& parameters,
                             QWidget*                         parent = 0);

private slots:
    void
    handleTreeAfterSource() override;

    void
    okButtonPressed() const override;

    void
    startSearchingThread();

    void
    enableOkButton() override;

    void
    populateTreeWidget() override;

private:
    QPointer<QPushButton> m_refreshButton;
    QPointer<QWidget> m_loadedInfoWidget;
    QPointer<LoadingWidget> m_loadingWidget;

    QPointer<TopicsServicesThread> m_thread;

    Parameters::RecordBagParameters& m_parameters;

    RecordBagSettings m_settings;

    static constexpr int HEIGHT_OFFSET = 80;
};
