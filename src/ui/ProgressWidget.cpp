#include "ProgressWidget.hpp"

#include "BagToImagesThread.hpp"
#include "BagToPCDsThread.hpp"
#include "BagToVideoThread.hpp"
#include "BasicThread.hpp"
#include "DialogSettings.hpp"
#include "ChangeCompressionBagThread.hpp"
#include "DummyBagThread.hpp"
#include "EditBagThread.hpp"
#include "MergeBagsThread.hpp"
#include "PCDsToBagThread.hpp"
#include "PublishImagesThread.hpp"
#include "PublishVideoThread.hpp"
#include "RecordBagThread.hpp"
#include "SendTF2Thread.hpp"
#include "BagTF2ToFileThread.hpp"
#include "VideoToBagThread.hpp"

#include <QLabel>
#include <QMessageBox>
#include <QMovie>
#include <QProgressBar>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

ProgressWidget::ProgressWidget(const QString& headerLabelText, Parameters::BasicParameters& parameters,
                               const Utils::UI::TOOL_ID threadTypeId, QWidget *parent) :
    QWidget(parent)
{
    switch (threadTypeId) {
    case Utils::UI::TOOL_ID::BAG_TO_VIDEO:
        m_thread = new BagToVideoThread(dynamic_cast<Parameters::BagToVideoParameters&>(parameters),
                                        DialogSettings::getStaticParameter("hw_acc", false), this);
        break;
    case Utils::UI::TOOL_ID::VIDEO_TO_BAG:
        m_thread = new VideoToBagThread(dynamic_cast<Parameters::VideoToBagParameters&>(parameters),
                                        DialogSettings::getStaticParameter("hw_acc", false), this);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_PCDS:
        m_thread = new BagToPCDsThread(dynamic_cast<Parameters::AdvancedParameters&>(parameters),
                                       DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_ID::PCDS_TO_BAG:
        m_thread = new PCDsToBagThread(dynamic_cast<Parameters::PCDsToBagParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_ID::BAG_TO_IMAGES:
        m_thread = new BagToImagesThread(dynamic_cast<Parameters::BagToImagesParameters&>(parameters),
                                         DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_ID::TF2_TO_FILE:
        m_thread = new BagTF2ToFileThread(dynamic_cast<Parameters::TF2ToFileParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_ID::EDIT_BAG:
        m_thread = new EditBagThread(dynamic_cast<Parameters::EditBagParameters&>(parameters),
                                     DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_ID::MERGE_BAGS:
        m_thread = new MergeBagsThread(dynamic_cast<Parameters::MergeBagsParameters&>(parameters),
                                       DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_ID::RECORD_BAG:
        m_thread = new RecordBagThread(dynamic_cast<Parameters::RecordBagParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_ID::DUMMY_BAG:
        m_thread = new DummyBagThread(dynamic_cast<Parameters::DummyBagParameters&>(parameters),
                                      DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_ID::COMPRESS_BAG:
        m_thread = new ChangeCompressionBagThread(dynamic_cast<Parameters::CompressBagParameters&>(parameters),
                                                  DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()),
                                                  true, this);
        break;
    case Utils::UI::TOOL_ID::DECOMPRESS_BAG:
        m_thread = new ChangeCompressionBagThread(dynamic_cast<Parameters::CompressBagParameters&>(parameters),
                                                  DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()),
                                                  false, this);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_VIDEO:
        m_thread = new PublishVideoThread(dynamic_cast<Parameters::PublishParameters&>(parameters),
                                          DialogSettings::getStaticParameter("hw_acc", false), this);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_IMAGES:
        m_thread = new PublishImagesThread(dynamic_cast<Parameters::PublishParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_ID::SEND_TF2:
        m_thread = new SendTF2Thread(dynamic_cast<Parameters::SendTF2Parameters&>(parameters), this);
        break;
    default:
        break;
    }

    const auto isDarkMode = Utils::UI::isDarkMode();

    auto* const headerLabel = new QLabel(headerLabelText);
    Utils::UI::setWidgetFontSize(headerLabel);
    headerLabel->setAlignment(Qt::AlignHCenter);

    auto* const progressLabel = new QLabel;
    progressLabel->setAlignment(Qt::AlignHCenter);

    auto* const cancelButton = new QPushButton(threadTypeId == Utils::UI::TOOL_ID::PUBLISH_VIDEO ||
                                               threadTypeId == Utils::UI::TOOL_ID::PUBLISH_IMAGES ||
                                               threadTypeId == Utils::UI::TOOL_ID::RECORD_BAG ? "Stop" : "Cancel");
    auto* const finishedButton = new QPushButton("Done");
    finishedButton->setVisible(false);

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(cancelButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(finishedButton);

    QWidget* progressWidget;
    const auto setMovie = [this, &progressWidget, progressLabel, isDarkMode] (const QString& moviePath, int width, int height) {
        m_movie = new QMovie(isDarkMode ? moviePath + "_white.gif" : moviePath + "_black.gif");
        m_movie->setScaledSize(QSize(width, height));

        auto* const movieLabel = new QLabel;
        movieLabel->setMovie(m_movie);
        movieLabel->setAlignment(Qt::AlignHCenter);
        progressWidget = movieLabel;

        connect(m_thread, &BasicThread::finished, this, [this, progressLabel] {
            m_movie->stop();
            progressLabel->setText("Done!");
        });

        m_movie->start();
    };

    // Display a progress bar or play a gif
    switch (threadTypeId) {
    case Utils::UI::TOOL_ID::MERGE_BAGS:
        progressLabel->setText("Writing merged Bag File...");
        setMovie(":/gifs/merging", 120, 110);
        break;
    case Utils::UI::TOOL_ID::COMPRESS_BAG:
        progressLabel->setText("Writing and compressing Bag File...");
        setMovie(":/gifs/compressing", 55, 120);
        break;
    case Utils::UI::TOOL_ID::DECOMPRESS_BAG:
        progressLabel->setText("Decompressing and writing new Bag File...");
        setMovie(":/gifs/decompressing", 55, 120);
        break;
    case Utils::UI::TOOL_ID::RECORD_BAG:
        progressLabel->setText("Recording Bag File...");
        setMovie(":/gifs/recording", 120, 70);
        break;
    case Utils::UI::TOOL_ID::PUBLISH_VIDEO:
    case Utils::UI::TOOL_ID::PUBLISH_IMAGES:
        setMovie(":/gifs/publishing", 120, 100);
        break;
    case Utils::UI::TOOL_ID::SEND_TF2:
        setMovie(":/gifs/sending_tf2", 100, 100);
        break;
    default:
        auto* const progressBar = new QProgressBar;
        progressBar->setVisible(false);
        progressWidget = progressBar;

        connect(m_thread, &BasicThread::progressChanged, this, [progressBar] (const QString& /*progressString*/, int progress) {
            if (!progressBar->isVisible()) {
                progressBar->setVisible(true);
            }
            progressBar->setValue(progress);
        });
        break;
    }

    auto* const uiLayout = new QVBoxLayout;
    uiLayout->addStretch();
    uiLayout->addWidget(headerLabel);
    uiLayout->addSpacing(50);
    uiLayout->addWidget(progressWidget);
    uiLayout->addWidget(progressLabel);
    uiLayout->addStretch();

    auto* const uiLayoutStretched = new QHBoxLayout;
    uiLayoutStretched->addStretch();
    uiLayoutStretched->addLayout(uiLayout);
    uiLayoutStretched->addStretch();

    auto* const mainLayout = new QVBoxLayout;
    mainLayout->addLayout(uiLayoutStretched);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    auto* const doneShortCut = new QShortcut(QKeySequence(Qt::Key_Return), this);

    connect(cancelButton, &QPushButton::clicked, this, [this] {
        if (m_thread->isRunning()) {
            m_thread->requestInterruption();
            m_thread->wait();
        }
        emit progressStopped();
    });
    connect(finishedButton, &QPushButton::clicked, this, [this] {
        emit finished();
    });
    connect(doneShortCut, &QShortcut::activated, this, [this, finishedButton] {
        if (!finishedButton->isVisible()) {
            return;
        }
        emit finished();
    });

    connect(m_thread, &BasicThread::informOfGatheringData, this, [progressLabel] () {
        progressLabel->setText("Collecting necessary data...");
    });
    connect(m_thread, &BasicThread::progressChanged, this, [progressLabel] (const QString& progressString, int /* progress */) {
        progressLabel->setText(progressString);
    });
    connect(m_thread, &BasicThread::finished, this, [cancelButton, finishedButton] {
        cancelButton->setVisible(false);
        finishedButton->setVisible(true);
    });
    connect(m_thread, &BasicThread::failed, this, [this] {
        auto* const messageBox = new QMessageBox(QMessageBox::Warning, "Failed processing files!",
                                                 "The file processing failed. Please make sure that all input parameters are set correctly, "
                                                 "that the input data is valid and disable the hardware acceleration, if necessary.");
        messageBox->exec();
        emit progressStopped();
    });
}


ProgressWidget::~ProgressWidget()
{
    m_thread->requestInterruption();
    m_thread->wait();
}


void
ProgressWidget::startThread()
{
    m_thread->start();
}


bool
ProgressWidget::event(QEvent *event)
{
    [[unlikely]] if ((event->type() == QEvent::ApplicationPaletteChange || event->type() == QEvent::PaletteChange) && m_movie) {
        m_movie->stop();

        auto movieFileName = m_movie->fileName();
        movieFileName.replace(movieFileName.contains("_black") ? "_black" : "_white",
                              movieFileName.contains("_black") ? "_white" : "_black");
        m_movie->setFileName(movieFileName);

        m_movie->start();
    }
    return QWidget::event(event);
}
