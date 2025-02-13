#include "MainWindow.hpp"

#include "BagInfoWidget.hpp"
#include "BagToImagesWidget.hpp"
#include "BagToVideoWidget.hpp"
#include "DummyBagWidget.hpp"
#include "EditBagWidget.hpp"
#include "MergeBagsWidget.hpp"
#include "ProgressWidget.hpp"
#include "PublishWidget.hpp"
#include "StartWidget.hpp"
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
    case 0:
        basicInputWidget = new BagToVideoWidget(m_parametersBagToVideo);
        break;
    case 1:
        basicInputWidget = new VideoToBagWidget(m_parametersVideoToBag, m_dialogParameters.checkROS2NameConform);
        break;
    case 2:
        basicInputWidget = new BagToImagesWidget(m_parametersBagToImages);
        break;
    case 3:
        basicInputWidget = new EditBagWidget(m_editBagParameters, m_dialogParameters.checkROS2NameConform);
        break;
    case 4:
        basicInputWidget = new MergeBagsWidget(m_mergeBagsParameters);
        break;
    case 5:
        basicInputWidget = new DummyBagWidget(m_dummyBagParameters, m_dialogParameters.checkROS2NameConform);
        break;
    case 6:
        basicInputWidget = new BagInfoWidget;
        break;
    case 7:
        basicInputWidget = new PublishWidget(m_publishParametersVideo, m_dialogParameters.checkROS2NameConform, true);
        break;
    case 8:
        basicInputWidget = new PublishWidget(m_publishParametersImages, m_dialogParameters.checkROS2NameConform, false);
        break;
    }

    resize(mode == 3 || mode == 4 ? basicInputWidget->width() : DEFAULT_WIDTH,
           mode == 3 || mode == 4 ? basicInputWidget->height() : DEFAULT_HEIGHT);
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
    case 0:
        progressWidget = new ProgressWidget(":/icons/bag_to_video_black.svg", ":/icons/bag_to_video_white.svg",
                                            "Encoding Video...", m_parametersBagToVideo, mode);
        break;
    case 1:
        progressWidget = new ProgressWidget(":/icons/video_to_bag_black.svg", ":/icons/video_to_bag_white.svg",
                                            "Writing to Bag...", m_parametersVideoToBag, mode);
        break;
    case 2:
        progressWidget = new ProgressWidget(":/icons/bag_to_images_black.svg", ":/icons/bag_to_images_white.svg",
                                            "Writing Images...", m_parametersBagToImages, mode);
        break;
    case 3:
        progressWidget = new ProgressWidget(":/icons/edit_bag_black.svg", ":/icons/edit_bag_white.svg",
                                            "Writing to edited ROSBag...", m_editBagParameters, mode);
        break;
    case 4:
        progressWidget = new ProgressWidget(":/icons/merge_bags_black.svg", ":/icons/merge_bags_white.svg",
                                            "Writing merged ROSBag...", m_mergeBagsParameters, mode);
        break;
    case 5:
        progressWidget = new ProgressWidget(":/icons/dummy_bag_black.svg", ":/icons/dummy_bag_white.svg",
                                            "Creating Bag...", m_dummyBagParameters, mode);
        break;
    case 7:
        progressWidget = new ProgressWidget(":/icons/publish_video_black.svg", ":/icons/publish_video_white.svg",
                                            "Publishing Video...", m_publishParametersVideo, mode);
        break;
    case 8:
        progressWidget = new ProgressWidget(":/icons/publish_images_black.svg", ":/icons/publish_images_white.svg",
                                            "Publishing Images...", m_publishParametersImages, mode);
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
