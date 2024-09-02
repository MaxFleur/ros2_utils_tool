#pragma once

#include "UtilsUI.hpp"

#include <QMainWindow>

/**
 * @brief Main window displaying the main user interface
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow();

private slots:
    void
    setStartWidget();

    void
    setConfigWidget(int mode);

    void
    setProgressWidget(int mode);

private:
    void
    closeEvent(QCloseEvent *event) override;

private:
    Utils::UI::VideoParameters m_parametersBagToVideo;
    Utils::UI::VideoParameters m_parametersVideoToBag;
    Utils::UI::ImageParameters m_parametersBagToImages;
    Utils::UI::PlayBagParameters m_parametersPlayBag;

    // We only need this one for encoding a video, but not for writing to a bag file
    QString m_encodingFormat = "mp4";
};
