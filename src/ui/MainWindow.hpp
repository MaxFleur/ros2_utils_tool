#pragma once

#include "Parameters.hpp"

#include <QMainWindow>

// Main window displaying the main user interface. The basic workflow is as follows:
// The main window uses the start widget to show all availabe tools. If the start widget is called,
// this will call a corresponding input widget where a user can modify all necessary parameters.
// If done, the input widget will be replaced with a progress widget showing the current progress.
// The progress widget calls a separate thread performing the main operation in the background.
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow();

private slots:
    // Used to switch between start, input and progress widget
    void
    setStartWidget();

    void
    setInputWidget(int mode);

    void
    setProgressWidget(int mode);

private:
    void
    closeEvent(QCloseEvent *event) override;

private:
    // Parameters storing all configurations done by a user in the input widgets.
    // The parameters are transferred to the progress widget and their thread.
    Parameters::BagToVideoParameters m_parametersBagToVideo;
    Parameters::VideoToBagParameters m_parametersVideoToBag;
    Parameters::AdvancedParameters m_parametersBagToPCDs;
    Parameters::PCDsToBagParameters m_parametersPCDsToBag;
    Parameters::BagToImagesParameters m_parametersBagToImages;
    Parameters::EditBagParameters m_parametersEditBag;
    Parameters::MergeBagsParameters m_parametersMergeBags;
    Parameters::DummyBagParameters m_parametersDummyBag;
    Parameters::CompressBagParameters m_parametersCompressBag;

    Parameters::PublishParameters m_parametersPublishVideo;
    Parameters::PublishParameters m_parametersPublishImages;
    // Parameters for settings dialog
    Parameters::DialogParameters m_dialogParameters;

    static constexpr int DEFAULT_WIDTH = 450;
    static constexpr int DEFAULT_HEIGHT = 600;
};
