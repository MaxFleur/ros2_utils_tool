#include "MainWindow.hpp"

#include "BagInfoWidget.hpp"
#include "BagToPCDsWidget.hpp"
#include "BagToImagesWidget.hpp"
#include "BagToVideoWidget.hpp"
#include "BagToYamlWidget.hpp"
#include "ChangeCompressionWidget.hpp"
#include "ConfigurePlayBagWidget.hpp"
#include "ConfigureRecordBagWidget.hpp"
#include "ControlBagWidget.hpp"
#include "ControlPlayBagWidget.hpp"
#include "ControlRecordBagWidget.hpp"
#include "DummyBagWidget.hpp"
#include "EditBagWidget.hpp"
#include "MergeBagsWidget.hpp"
#include "PCDsToBagWidget.hpp"
#include "ProgressWidget.hpp"
#include "PublishWidget.hpp"
#include "SendTF2Widget.hpp"
#include "StartWidget.hpp"
#include "BagTF2ToFileWidget.hpp"
#include "TopicsServicesInfoWidget.hpp"
#include "VideoToBagWidget.hpp"

#include "DialogSettings.hpp"

#include <QCloseEvent>
#include <QScrollArea>

#include <csignal>

MainWindow::MainWindow()
{
    setWindowTitle("ROS2 Utils Tool");
    // We need to get some values without having to access the dialog beforehand
    DialogSettings settings(m_dialogParameters, "dialog");

    setStartWidget();
}


void
MainWindow::setStartWidget()
{
    auto* const startWidget = new StartWidget(m_dialogParameters);

    // If we have many buttons we need a scrollbar
    auto* const scrollArea = new QScrollArea;
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidget(startWidget);
    // Make it so that only the scrollbar is visible
    auto palette = scrollArea->palette();
    palette.setColor(QPalette::Base, Qt::transparent);
    scrollArea->setPalette(palette);

    connect(startWidget, &StartWidget::toolRequested, this, &MainWindow::setInputWidget);
    resizeToDefault(scrollArea);
}


void
MainWindow::setInputWidget(Utils::UI::TOOL_ID mode)
{
    QPointer<BasicInputWidget> basicInputWidget;
    switch (mode) {
    case Utils::UI::TOOL_ID::BAG_TO_VIDEO:
        basicInputWidget = new BagToVideoWidget(m_bagToVideoParameters);
        break;
    case Utils::UI::TOOL_ID::VIDEO_TO_BAG:
        basicInputWidget = new VideoToBagWidget(m_videoToBagParameters, m_dialogParameters.usePredefinedTopicNames,
                                                m_dialogParameters.warnROS2NameConvention);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_PCDS:
        basicInputWidget = new BagToPCDsWidget(m_bagToPCDsParameters);
        break;
    case Utils::UI::TOOL_ID::PCDS_TO_BAG:
        basicInputWidget = new PCDsToBagWidget(m_PCDsToBagParameters, m_dialogParameters.usePredefinedTopicNames,
                                               m_dialogParameters.warnROS2NameConvention);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_IMAGES:
        basicInputWidget = new BagToImagesWidget(m_bagToImagesParameters);
        break;
    case Utils::UI::TOOL_ID::TF2_TO_FILE:
        basicInputWidget = new BagTF2ToFileWidget(m_TF2ToFileParameters);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_YAML:
        basicInputWidget = new BagToYamlWidget(m_bagToYamlParameters);
        break;
    case Utils::UI::TOOL_ID::EDIT_BAG:
        basicInputWidget = new EditBagWidget(m_editBagParameters, m_dialogParameters.warnROS2NameConvention);
        break;
    case Utils::UI::TOOL_ID::MERGE_BAGS:
        basicInputWidget = new MergeBagsWidget(m_mergeBagsParameters);
        break;
    case Utils::UI::TOOL_ID::RECORD_BAG:
        basicInputWidget = new ConfigureRecordBagWidget(m_recordBagParameters);
        break;
    case Utils::UI::TOOL_ID::DUMMY_BAG:
        basicInputWidget = new DummyBagWidget(m_dummyBagParameters, m_dialogParameters.warnROS2NameConvention);
        break;
    case Utils::UI::TOOL_ID::COMPRESS_BAG:
        basicInputWidget = new ChangeCompressionWidget(m_compressBagParameters, true);
        break;
    case Utils::UI::TOOL_ID::DECOMPRESS_BAG:
        basicInputWidget = new ChangeCompressionWidget(m_decompressBagParameters, false);
        break;
    case Utils::UI::TOOL_ID::PLAY_BAG:
        basicInputWidget = new ConfigurePlayBagWidget(m_playBagParameters);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_VIDEO:
        basicInputWidget = new PublishWidget(m_publishVideoParameters, m_dialogParameters.usePredefinedTopicNames,
                                             m_dialogParameters.warnROS2NameConvention, true);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_IMAGES:
        basicInputWidget = new PublishWidget(m_publishImagesParameters, m_dialogParameters.usePredefinedTopicNames,
                                             m_dialogParameters.warnROS2NameConvention, false);
        break;
    case Utils::UI::TOOL_ID::SEND_TF2:
        basicInputWidget = new SendTF2Widget(m_parametersSendTF2);
        break;
    case Utils::UI::TOOL_ID::TOPICS_SERVICES_INFO:
        basicInputWidget = new TopicsServicesInfoWidget;
        break;
    case Utils::UI::TOOL_ID::BAG_INFO:
        basicInputWidget = new BagInfoWidget;
        break;
    }

    connect(basicInputWidget, &BasicInputWidget::back, this, &MainWindow::setStartWidget);
    connect(basicInputWidget, &BasicInputWidget::okPressed, this, [this, mode] {
        setProcessingWidget(mode);
    });

    resizeToDefault(basicInputWidget);
}


void
MainWindow::setProcessingWidget(Utils::UI::TOOL_ID mode)
{
    // Doesn't make sense to show any progress when we want to actively control things
    // So use a control widget instead
    if (mode == Utils::UI::TOOL_ID::PLAY_BAG || mode == Utils::UI::TOOL_ID::RECORD_BAG) {
        QPointer<ControlBagWidget> controlBagWidget;
        if (mode == Utils::UI::TOOL_ID::PLAY_BAG) {
            controlBagWidget = new ControlPlayBagWidget(m_playBagParameters);
        } else {
            controlBagWidget = new ControlRecordBagWidget(m_recordBagParameters);
        }

        resizeToDefault(controlBagWidget);
        connect(controlBagWidget, &ControlBagWidget::stopped, this, [this, mode] {
            setInputWidget(mode);
        });

        return;
    }

    QPointer<ProgressWidget> progressWidget;
    switch (mode) {
    case Utils::UI::TOOL_ID::BAG_TO_VIDEO:
        progressWidget = new ProgressWidget("Encoding Video...", m_bagToVideoParameters, mode);
        break;
    case Utils::UI::TOOL_ID::VIDEO_TO_BAG:
        progressWidget = new ProgressWidget("Writing to Bag...", m_videoToBagParameters, mode);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_PCDS:
        progressWidget = new ProgressWidget("Writing PCD files...", m_bagToPCDsParameters, mode);
        break;
    case Utils::UI::TOOL_ID::PCDS_TO_BAG:
        progressWidget = new ProgressWidget("Writing to Bag...", m_PCDsToBagParameters, mode);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_IMAGES:
        progressWidget = new ProgressWidget("Writing Images...", m_bagToImagesParameters, mode);
        break;
    case Utils::UI::TOOL_ID::TF2_TO_FILE:
        progressWidget = new ProgressWidget("Writing File(s)...", m_TF2ToFileParameters, mode);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_YAML:
        progressWidget = new ProgressWidget("Writing File(s)...", m_bagToYamlParameters, mode);
        break;
    case Utils::UI::TOOL_ID::EDIT_BAG:
        progressWidget = new ProgressWidget("Writing edited Bag File...", m_editBagParameters, mode);
        break;
    case Utils::UI::TOOL_ID::MERGE_BAGS:
        progressWidget = new ProgressWidget("Writing merged Bag File...", m_mergeBagsParameters, mode);
        break;
    case Utils::UI::TOOL_ID::DUMMY_BAG:
        progressWidget = new ProgressWidget("Creating Bag...", m_dummyBagParameters, mode);
        break;
    case Utils::UI::TOOL_ID::COMPRESS_BAG:
        progressWidget = new ProgressWidget("Compressing Bag...", m_compressBagParameters, mode);
        break;
    case Utils::UI::TOOL_ID::DECOMPRESS_BAG:
        progressWidget = new ProgressWidget("Decompressing Bag...", m_decompressBagParameters, mode);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_VIDEO:
        progressWidget = new ProgressWidget("Publishing Video...", m_publishVideoParameters, mode);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_IMAGES:
        progressWidget = new ProgressWidget("Publishing Images...", m_publishImagesParameters, mode);
        break;
    case Utils::UI::TOOL_ID::SEND_TF2:
        // The input widget contains a ROS node, which would still be active if we create the progress widget
        // with an instance of the send tf2 thread. To avoid two nodes existing, destroy the input widget containg the first node
        delete centralWidget();
        progressWidget = new ProgressWidget("Sending TF2...", m_parametersSendTF2, mode);
        break;
    default:
        break;
    }

    connect(progressWidget, &ProgressWidget::stopped, this, [this, mode] {
        setInputWidget(mode);
    });
    connect(progressWidget, &ProgressWidget::finished, this, &MainWindow::setStartWidget);

    resizeToDefault(progressWidget);
    progressWidget->startThread();
}


void
MainWindow::resizeToDefault(QWidget* widget)
{
    static constexpr int DEFAULT_WIDTH = 450;
    static constexpr int DEFAULT_HEIGHT = 600;

    setCentralWidget(widget);
    layout()->activate();
    resize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
}


void
MainWindow::closeEvent(QCloseEvent *event)
{
    std::raise(SIGINT);
    event->accept();
}
