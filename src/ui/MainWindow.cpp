#include "MainWindow.hpp"

#include "BagToImagesWidget.hpp"
#include "BagToVideoWidget.hpp"
#include "ImagesProgressWidget.hpp"
#include "PlayBagConfigWidget.hpp"
#include "PlayBagProgressWidget.hpp"
#include "StartWidget.hpp"
#include "VideoToBagWidget.hpp"
#include "VideoProgressWidget.hpp"

#include <QApplication>
#include <QCloseEvent>

#include "rclcpp/rclcpp.hpp"

MainWindow::MainWindow()
{
    setWindowTitle("ROS Tools");

    setStartWidget();

    resize(450, 450);
}


void
MainWindow::setStartWidget()
{
    auto* const startWidget = new StartWidget;
    connect(startWidget, &StartWidget::bagToVideoRequested, this, [this] {
        setConfigWidget(0);
    });
    connect(startWidget, &StartWidget::videoToBagRequested, this, [this] {
        setConfigWidget(1);
    });
    connect(startWidget, &StartWidget::bagToImagesRequested, this, [this] {
        setConfigWidget(2);
    });
    connect(startWidget, &StartWidget::playBagRequested, this, [this] {
        setConfigWidget(3);
    });
    setCentralWidget(startWidget);
}


void
MainWindow::setConfigWidget(int mode)
{
    QPointer<BasicConfigWidget> basicConfigWidget;
    switch (mode) {
    case 0:
        basicConfigWidget = new BagToVideoWidget(m_parametersBagToVideo, m_encodingFormat);
        break;
    case 1:
        basicConfigWidget = new VideoToBagWidget(m_parametersVideoToBag);
        break;
    case 2:
        basicConfigWidget = new BagToImagesWidget(m_parametersBagToImages);
        break;
    case 3:
        basicConfigWidget = new PlayBagConfigWidget(m_parametersPlayBag);
        break;
    }
    setCentralWidget(basicConfigWidget);

    connect(basicConfigWidget, &BasicConfigWidget::back, this, &MainWindow::setStartWidget);
    connect(basicConfigWidget, &BasicConfigWidget::okPressed, this, [this, mode] {
        setProgressWidget(mode);
    });
}


void
MainWindow::setProgressWidget(int mode)
{
    if (mode == 3) {
        auto* const progressWidget = new PlayBagProgressWidget(m_parametersPlayBag);
        connect(progressWidget, &PlayBagProgressWidget::stopped, this, [this] (bool terminateApplication) {
            if (terminateApplication) {
                rclcpp::shutdown();
                QApplication::quit();
            } else {
                setStartWidget();
            }
        });

        setCentralWidget(progressWidget);
        progressWidget->startThread();
    } else {
        QPointer<BasicProgressWidget> progressWidget;
        switch (mode) {
        case 0:
        case 1:
            progressWidget = new VideoProgressWidget(mode == 0 ? m_parametersBagToVideo : m_parametersVideoToBag, mode == 0);
            break;
        case 2:
            progressWidget = new ImagesProgressWidget(m_parametersBagToImages);
            break;
        }

        connect(progressWidget, &BasicProgressWidget::progressStopped, this, [this, mode] {
            setConfigWidget(mode);
        });
        connect(progressWidget, &BasicProgressWidget::finished, this, &MainWindow::setStartWidget);

        setCentralWidget(progressWidget);
        progressWidget->startThread();
    }
}


void
MainWindow::closeEvent(QCloseEvent *event)
{
    rclcpp::shutdown();
    event->accept();
}
