#pragma once

#include "UtilsUI.hpp"

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
    Utils::UI::VideoInputParameters m_parametersBagToVideo;
    Utils::UI::BagInputParameters m_parametersVideoToBag;
    Utils::UI::AdvancedInputParameters m_parametersBagToPCDs;
    Utils::UI::ImageInputParameters m_parametersBagToImages;
    Utils::UI::EditBagInputParameters m_editBagParameters;
    Utils::UI::MergeBagsInputParameters m_mergeBagsParameters;
    Utils::UI::DummyBagInputParameters m_dummyBagParameters;

    Utils::UI::PublishParameters m_publishParametersVideo;
    Utils::UI::PublishParameters m_publishParametersImages;
    // Parameters for settings dialog
    Utils::UI::DialogParameters m_dialogParameters;

    static constexpr int DEFAULT_WIDTH = 450;
    static constexpr int DEFAULT_HEIGHT = 600;
};
