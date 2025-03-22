#include "ProgressWidget.hpp"

#include "BagToImagesThread.hpp"
#include "BagToPCDsThread.hpp"
#include "BagToVideoThread.hpp"
#include "BasicThread.hpp"
#include "DialogSettings.hpp"
#include "CompressBagThread.hpp"
#include "DummyBagThread.hpp"
#include "EditBagThread.hpp"
#include "MergeBagsThread.hpp"
#include "PCDsToBagThread.hpp"
#include "PublishImagesThread.hpp"
#include "PublishVideoThread.hpp"
#include "UtilsUI.hpp"
#include "VideoToBagThread.hpp"

#include <QLabel>
#include <QMessageBox>
#include <QMovie>
#include <QProgressBar>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

ProgressWidget::ProgressWidget(const QString& headerPixmapLabelTextBlack, const QString& headerPixmapLabelTextWhite,
                               const QString& headerLabelText, Parameters::BasicParameters& parameters,
                               const int threadTypeId, QWidget *parent) :
    QWidget(parent)
{
    switch (threadTypeId) {
    case Utils::UI::TOOL_BAG_TO_VIDEO:
        m_thread = new BagToVideoThread(dynamic_cast<Parameters::BagToVideoParameters&>(parameters),
                                        DialogSettings::getStaticParameter("hw_acc", false), this);
        break;
    case Utils::UI::TOOL_VIDEO_TO_BAG:
        m_thread = new VideoToBagThread(dynamic_cast<Parameters::VideoToBagParameters&>(parameters),
                                        DialogSettings::getStaticParameter("hw_acc", false), this);
        break;
    case Utils::UI::TOOL_BAG_TO_PCDS:
        m_thread = new BagToPCDsThread(dynamic_cast<Parameters::AdvancedParameters&>(parameters),
                                       DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_PCDS_TO_BAG:
        m_thread = new PCDsToBagThread(dynamic_cast<Parameters::PCDsToBagParameters&>(parameters),
                                       DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_BAG_TO_IMAGES:
        m_thread = new BagToImagesThread(dynamic_cast<Parameters::BagToImagesParameters&>(parameters),
                                         DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_EDIT_BAG:
        m_thread = new EditBagThread(dynamic_cast<Parameters::EditBagParameters&>(parameters),
                                     DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_MERGE_BAGS:
        m_thread = new MergeBagsThread(dynamic_cast<Parameters::MergeBagsParameters&>(parameters),
                                       DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_DUMMY_BAG:
        m_thread = new DummyBagThread(dynamic_cast<Parameters::DummyBagParameters&>(parameters),
                                      DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_COMPRESS_BAG:
        m_thread = new CompressBagThread(dynamic_cast<Parameters::CompressBagParameters&>(parameters),
                                         DialogSettings::getStaticParameter("max_threads", std::thread::hardware_concurrency()), this);
        break;
    case Utils::UI::TOOL_PUBLISH_VIDEO:
        m_thread = new PublishVideoThread(dynamic_cast<Parameters::PublishParameters&>(parameters),
                                          DialogSettings::getStaticParameter("hw_acc", false), this);
        break;
    case Utils::UI::TOOL_PUBLISH_IMAGES:
        m_thread = new PublishImagesThread(dynamic_cast<Parameters::PublishParameters&>(parameters), this);
        break;
    }

    const auto isDarkMode = Utils::UI::isDarkMode();

    auto* const headerPixmapLabel = new QLabel;
    headerPixmapLabel->setPixmap(QIcon(isDarkMode ? headerPixmapLabelTextWhite : headerPixmapLabelTextBlack).pixmap(QSize(100, 45)));
    headerPixmapLabel->setAlignment(Qt::AlignHCenter);

    auto* const headerLabel = new QLabel(headerLabelText);
    Utils::UI::setWidgetFontSize(headerLabel);
    headerLabel->setAlignment(Qt::AlignHCenter);

    auto* const progressLabel = new QLabel;
    progressLabel->setAlignment(Qt::AlignHCenter);

    auto* const cancelButton = new QPushButton("Cancel");
    auto* const finishedButton = new QPushButton("Done");
    finishedButton->setVisible(false);

    auto* const buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(cancelButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(finishedButton);

    // Display a progress bar or play a gif depending on if we are doing bag or publishing stuff
    QWidget* progressWidget;
    if (threadTypeId == Utils::UI::TOOL_COMPRESS_BAG || threadTypeId == Utils::UI::TOOL_MERGE_BAGS ||
        threadTypeId == Utils::UI::TOOL_PUBLISH_VIDEO || threadTypeId == Utils::UI::TOOL_PUBLISH_IMAGES) {
        QMovie* movie;
        if (threadTypeId == Utils::UI::TOOL_COMPRESS_BAG || threadTypeId == Utils::UI::TOOL_MERGE_BAGS) {
            movie = new QMovie(isDarkMode? ":/gifs/processing_white.gif" : ":/gifs/processing_black.gif");
        } else {
            movie = new QMovie(isDarkMode? ":/gifs/publishing_white.gif" : ":/gifs/publishing_black.gif");
        }
        movie->setScaledSize(QSize(120, threadTypeId == Utils::UI::TOOL_COMPRESS_BAG ? 120 : 100));

        auto* const movieLabel = new QLabel;
        movieLabel->setMovie(movie);
        movieLabel->setAlignment(Qt::AlignHCenter);
        progressWidget = movieLabel;

        connect(m_thread, &BasicThread::finished, this, [movie, progressLabel] {
            movie->stop();
            progressLabel->setText("Done!");
        });

        movie->start();
    } else {
        auto* const progressBar = new QProgressBar;
        progressBar->setVisible(false);
        progressWidget = progressBar;

        connect(m_thread, &BasicThread::progressChanged, this, [progressBar] (const QString& /*progressString*/, int progress) {
            if (!progressBar->isVisible()) {
                progressBar->setVisible(true);
            }
            progressBar->setValue(progress);
        });
    }

    auto* const uiLayout = new QVBoxLayout;
    uiLayout->addStretch();
    uiLayout->addWidget(headerPixmapLabel);
    uiLayout->addWidget(headerLabel);
    uiLayout->addSpacing(30);
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
    connect(m_thread, &BasicThread::processing, this, [progressLabel] () {
        progressLabel->setText("Processing, this might take a while...");
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
