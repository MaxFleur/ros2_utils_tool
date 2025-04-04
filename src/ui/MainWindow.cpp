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
#include "StartWidget.hpp"
#include "UtilsUI.hpp"
#include "VideoToBagWidget.hpp"

#include "DialogSettings.hpp"

#include <QCloseEvent>
#include <QTimer>

#include <csignal>

MainWindow::MainWindow()
{
    setWindowTitle("ROS Tools");
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
MainWindow::setInputWidget(int mode)
{
    QPointer<BasicInputWidget> basicInputWidget;
    switch (mode) {
    case Utils::UI::TOOL_BAG_TO_VIDEO:
        basicInputWidget = new BagToVideoWidget(m_parametersBagToVideo);
        break;
    case Utils::UI::TOOL_VIDEO_TO_BAG:
        basicInputWidget = new VideoToBagWidget(m_parametersVideoToBag, m_dialogParameters.usePredefinedTopicNames,
                                                m_dialogParameters.checkROS2NameConform);
        break;
    case Utils::UI::TOOL_BAG_TO_PCDS:
        basicInputWidget = new BagToPCDsWidget(m_parametersBagToPCDs);
        break;
    case Utils::UI::TOOL_PCDS_TO_BAG:
        basicInputWidget = new PCDsToBagWidget(m_parametersPCDsToBag, m_dialogParameters.usePredefinedTopicNames,
                                               m_dialogParameters.checkROS2NameConform);
        break;
    case Utils::UI::TOOL_BAG_TO_IMAGES:
        basicInputWidget = new BagToImagesWidget(m_parametersBagToImages);
        break;
    case Utils::UI::TOOL_EDIT_BAG:
        basicInputWidget = new EditBagWidget(m_parametersEditBag, m_dialogParameters.checkROS2NameConform);
        break;
    case Utils::UI::TOOL_MERGE_BAGS:
        basicInputWidget = new MergeBagsWidget(m_parametersMergeBags);
        break;
    case Utils::UI::TOOL_COMPRESS_BAG:
        basicInputWidget = new ChangeCompressionWidget(m_parametersCompressBag, true);
        break;
    case Utils::UI::TOOL_DECOMPRESS_BAG:
        basicInputWidget = new ChangeCompressionWidget(m_parametersDecompressBag, false);
        break;
    case Utils::UI::TOOL_DUMMY_BAG:
        basicInputWidget = new DummyBagWidget(m_parametersDummyBag, m_dialogParameters.checkROS2NameConform);
        break;
    case Utils::UI::TOOL_PUBLISH_VIDEO:
        basicInputWidget = new PublishWidget(m_parametersPublishVideo, m_dialogParameters.usePredefinedTopicNames,
                                             m_dialogParameters.checkROS2NameConform, true);
        break;
    case Utils::UI::TOOL_PUBLISH_IMAGES:
        basicInputWidget = new PublishWidget(m_parametersPublishImages, m_dialogParameters.usePredefinedTopicNames,
                                             m_dialogParameters.checkROS2NameConform, false);
        break;
    case Utils::UI::TOOL_BAG_INFO:
        basicInputWidget = new BagInfoWidget;
        break;
    }

    resize(mode == Utils::UI::TOOL_EDIT_BAG || mode == Utils::UI::TOOL_MERGE_BAGS ? basicInputWidget->width() : DEFAULT_WIDTH,
           mode == Utils::UI::TOOL_EDIT_BAG || mode == Utils::UI::TOOL_MERGE_BAGS ? basicInputWidget->height() : DEFAULT_HEIGHT);
    setCentralWidget(basicInputWidget);

    connect(basicInputWidget, &BasicInputWidget::back, this, &MainWindow::setStartWidget);
    connect(basicInputWidget, &BasicInputWidget::okPressed, this, [this, mode] {
        setProgressWidget(mode);
    });
}


void
MainWindow::setProgressWidget(int mode)
{
    QPointer<ProgressWidget> progressWidget;
    switch (mode) {
    case Utils::UI::TOOL_BAG_TO_VIDEO:
        progressWidget = new ProgressWidget(":/icons/bag_to_video_black.svg", ":/icons/bag_to_video_white.svg",
                                            "Encoding Video...", m_parametersBagToVideo, mode);
        break;
    case Utils::UI::TOOL_VIDEO_TO_BAG:
        progressWidget = new ProgressWidget(":/icons/video_to_bag_black.svg", ":/icons/video_to_bag_white.svg",
                                            "Writing to Bag...", m_parametersVideoToBag, mode);
        break;
    case Utils::UI::TOOL_BAG_TO_PCDS:
        progressWidget = new ProgressWidget(":/icons/bag_to_pcd_black.svg", ":/icons/bag_to_pcd_white.svg",
                                            "Writing PCD files...", m_parametersBagToPCDs, mode);
        break;
    case Utils::UI::TOOL_PCDS_TO_BAG:
        progressWidget = new ProgressWidget(":/icons/pcd_to_bag_black.svg", ":/icons/pcd_to_bag_white.svg",
                                            "Writing to Bag...", m_parametersPCDsToBag, mode);
        break;
    case Utils::UI::TOOL_BAG_TO_IMAGES:
        progressWidget = new ProgressWidget(":/icons/bag_to_images_black.svg", ":/icons/bag_to_images_white.svg",
                                            "Writing Images...", m_parametersBagToImages, mode);
        break;
    case Utils::UI::TOOL_EDIT_BAG:
        progressWidget = new ProgressWidget(":/icons/edit_bag_black.svg", ":/icons/edit_bag_white.svg",
                                            "Writing edited bag file...", m_parametersEditBag, mode);
        break;
    case Utils::UI::TOOL_MERGE_BAGS:
        progressWidget = new ProgressWidget(":/icons/merge_bags_black.svg", ":/icons/merge_bags_white.svg",
                                            "Writing merged bag file...", m_parametersMergeBags, mode);
        break;
    case Utils::UI::TOOL_COMPRESS_BAG:
        progressWidget = new ProgressWidget(":/icons/compress_bag_black.svg", ":/icons/compress_bag_white.svg",
                                            "Compressing Bag...", m_parametersCompressBag, mode);
        break;
    case Utils::UI::TOOL_DECOMPRESS_BAG:
        progressWidget = new ProgressWidget(":/icons/decompress_bag_black.svg", ":/icons/decompress_bag_white.svg",
                                            "Decompressing Bag...", m_parametersDecompressBag, mode);
        break;
    case Utils::UI::TOOL_DUMMY_BAG:
        progressWidget = new ProgressWidget(":/icons/dummy_bag_black.svg", ":/icons/dummy_bag_white.svg",
                                            "Creating Bag...", m_parametersDummyBag, mode);
        break;
    case Utils::UI::TOOL_PUBLISH_VIDEO:
        progressWidget = new ProgressWidget(":/icons/publish_video_black.svg", ":/icons/publish_video_white.svg",
                                            "Publishing Video...", m_parametersPublishVideo, mode);
        break;
    case Utils::UI::TOOL_PUBLISH_IMAGES:
        progressWidget = new ProgressWidget(":/icons/publish_images_black.svg", ":/icons/publish_images_white.svg",
                                            "Publishing Images...", m_parametersPublishImages, mode);
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
