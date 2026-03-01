#pragma once

#include "Parameters.hpp"
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
    setInputWidget(Utils::UI::TOOL_ID mode);

    void
    setProcessingWidget(Utils::UI::TOOL_ID mode);

private:
    void
    closeEvent(QCloseEvent *event) override;

private:
    // Parameters storing all configurations done by a user in the input widgets.
    // The parameters are transferred to the progress widget and their thread.
    Parameters::BagToVideoParameters m_bagToVideoParameters;
    Parameters::VideoToBagParameters m_videoToBagParameters;
    Parameters::AdvancedParameters m_bagToPCDsParameters;
    Parameters::PCDsToBagParameters m_PCDsToBagParameters;
    Parameters::BagToImagesParameters m_bagToImagesParameters;
    Parameters::TF2ToFileParameters m_TF2ToFileParameters;
    Parameters::EditBagParameters m_editBagParameters;
    Parameters::MergeBagsParameters m_mergeBagsParameters;
    Parameters::RecordBagParameters m_recordBagParameters;
    Parameters::DummyBagParameters m_dummyBagParameters;
    Parameters::CompressBagParameters m_compressBagParameters;
    Parameters::CompressBagParameters m_decompressBagParameters;
    Parameters::PlayBagParameters m_playBagParameters;

    Parameters::PublishParameters m_publishVideoParameters;
    Parameters::PublishParameters m_publishImagesParameters;

    Parameters::SendTF2Parameters m_parametersSendTF2;
    // Parameters for settings dialog
    Parameters::DialogParameters m_dialogParameters;

    static constexpr int DEFAULT_WIDTH = 450;
    static constexpr int DEFAULT_HEIGHT = 600;
};
