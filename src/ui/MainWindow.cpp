#include "MainWindow.hpp"

#include "BagInfoWidget.hpp"
#include "BagToPCDsWidget.hpp"
#include "BagToImagesWidget.hpp"
#include "BagToVideoWidget.hpp"
#include "ChangeCompressionWidget.hpp"
#include "DummyBagWidget.hpp"
#include "EditBagWidget.hpp"
#include "MergeBagsWidget.hpp"
#include "PCDsToBagWidget.hpp"
#include "ProgressWidget.hpp"
#include "PublishWidget.hpp"
#include "RecordBagWidget.hpp"
#include "StartWidget.hpp"
#include "TF2ToJsonWidget.hpp"
#include "TopicsServicesInfoWidget.hpp"
#include "VideoToBagWidget.hpp"

#include "DialogSettings.hpp"

#include <QCloseEvent>
#include <QTimer>

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
    // Resize event is not called inside the function, so use a delay
    QTimer::singleShot(1, [this] {
        resize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
    });
    setCentralWidget(startWidget);
    connect(startWidget, &StartWidget::toolRequested, this, &MainWindow::setInputWidget);
}


void
MainWindow::setInputWidget(Utils::UI::TOOL_ID mode)
{
    QPointer<BasicInputWidget> basicInputWidget;
    switch (mode) {
    case Utils::UI::TOOL_ID::BAG_TO_VIDEO:
        basicInputWidget = new BagToVideoWidget(m_parametersBagToVideo);
        break;
    case Utils::UI::TOOL_ID::VIDEO_TO_BAG:
        basicInputWidget = new VideoToBagWidget(m_parametersVideoToBag, m_dialogParameters.usePredefinedTopicNames,
                                                m_dialogParameters.warnROS2NameConvention);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_PCDS:
        basicInputWidget = new BagToPCDsWidget(m_parametersBagToPCDs);
        break;
    case Utils::UI::TOOL_ID::PCDS_TO_BAG:
        basicInputWidget = new PCDsToBagWidget(m_parametersPCDsToBag, m_dialogParameters.usePredefinedTopicNames,
                                               m_dialogParameters.warnROS2NameConvention);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_IMAGES:
        basicInputWidget = new BagToImagesWidget(m_parametersBagToImages);
        break;
    case Utils::UI::TOOL_ID::TF2_TO_JSON:
        basicInputWidget = new TF2ToJsonWidget(m_parametersTF2ToJson);
        break;
    case Utils::UI::TOOL_ID::EDIT_BAG:
        basicInputWidget = new EditBagWidget(m_parametersEditBag, m_dialogParameters.warnROS2NameConvention);
        break;
    case Utils::UI::TOOL_ID::MERGE_BAGS:
        basicInputWidget = new MergeBagsWidget(m_parametersMergeBags);
        break;
    case Utils::UI::TOOL_ID::RECORD_BAG:
        basicInputWidget = new RecordBagWidget(m_parametersRecordBag);
        break;
    case Utils::UI::TOOL_ID::DUMMY_BAG:
        basicInputWidget = new DummyBagWidget(m_parametersDummyBag, m_dialogParameters.warnROS2NameConvention);
        break;
    case Utils::UI::TOOL_ID::COMPRESS_BAG:
        basicInputWidget = new ChangeCompressionWidget(m_parametersCompressBag, true);
        break;
    case Utils::UI::TOOL_ID::DECOMPRESS_BAG:
        basicInputWidget = new ChangeCompressionWidget(m_parametersDecompressBag, false);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_VIDEO:
        basicInputWidget = new PublishWidget(m_parametersPublishVideo, m_dialogParameters.usePredefinedTopicNames,
                                             m_dialogParameters.warnROS2NameConvention, true);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_IMAGES:
        basicInputWidget = new PublishWidget(m_parametersPublishImages, m_dialogParameters.usePredefinedTopicNames,
                                             m_dialogParameters.warnROS2NameConvention, false);
        break;
    case Utils::UI::TOOL_ID::TOPICS_SERVICES_INFO:
        basicInputWidget = new TopicsServicesInfoWidget;
        break;
    case Utils::UI::TOOL_ID::BAG_INFO:
        basicInputWidget = new BagInfoWidget;
        break;
    }

    resize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
    setCentralWidget(basicInputWidget);

    connect(basicInputWidget, &BasicInputWidget::back, this, &MainWindow::setStartWidget);
    connect(basicInputWidget, &BasicInputWidget::okPressed, this, [this, mode] {
        setProgressWidget(mode);
    });
}


void
MainWindow::setProgressWidget(Utils::UI::TOOL_ID mode)
{
    QPointer<ProgressWidget> progressWidget;
    switch (mode) {
    case Utils::UI::TOOL_ID::BAG_TO_VIDEO:
        progressWidget = new ProgressWidget("Encoding Video...", m_parametersBagToVideo, mode);
        break;
    case Utils::UI::TOOL_ID::VIDEO_TO_BAG:
        progressWidget = new ProgressWidget("Writing to Bag...", m_parametersVideoToBag, mode);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_PCDS:
        progressWidget = new ProgressWidget("Writing PCD files...", m_parametersBagToPCDs, mode);
        break;
    case Utils::UI::TOOL_ID::PCDS_TO_BAG:
        progressWidget = new ProgressWidget("Writing to Bag...", m_parametersPCDsToBag, mode);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_IMAGES:
        progressWidget = new ProgressWidget("Writing Images...", m_parametersBagToImages, mode);
        break;
    case Utils::UI::TOOL_ID::TF2_TO_JSON:
        progressWidget = new ProgressWidget("Writing Json File(s)...", m_parametersTF2ToJson, mode);
        break;
    case Utils::UI::TOOL_ID::EDIT_BAG:
        progressWidget = new ProgressWidget("Writing edited Bag File...", m_parametersEditBag, mode);
        break;
    case Utils::UI::TOOL_ID::MERGE_BAGS:
        progressWidget = new ProgressWidget("Writing merged Bag File...", m_parametersMergeBags, mode);
        break;
    case Utils::UI::TOOL_ID::RECORD_BAG:
        progressWidget = new ProgressWidget("Recording Bag File...", m_parametersRecordBag, mode);
        break;
    case Utils::UI::TOOL_ID::DUMMY_BAG:
        progressWidget = new ProgressWidget("Creating Bag...", m_parametersDummyBag, mode);
        break;
    case Utils::UI::TOOL_ID::COMPRESS_BAG:
        progressWidget = new ProgressWidget("Compressing Bag...", m_parametersCompressBag, mode);
        break;
    case Utils::UI::TOOL_ID::DECOMPRESS_BAG:
        progressWidget = new ProgressWidget("Decompressing Bag...", m_parametersDecompressBag, mode);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_VIDEO:
        progressWidget = new ProgressWidget("Publishing Video...", m_parametersPublishVideo, mode);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_IMAGES:
        progressWidget = new ProgressWidget("Publishing Images...", m_parametersPublishImages, mode);
        break;
    default:
        break;
    }
    // Resize event is not called inside the function, so use a delay
    QTimer::singleShot(1, [this] {
        resize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
    });
    setCentralWidget(progressWidget);

    connect(progressWidget, &ProgressWidget::progressStopped, this, [this, mode] {
        setInputWidget(mode);
    });
    connect(progressWidget, &ProgressWidget::finished, this, &MainWindow::setStartWidget);

    progressWidget->startThread();
}


void
MainWindow::closeEvent(QCloseEvent *event)
{
    std::raise(SIGINT);
    event->accept();
}
