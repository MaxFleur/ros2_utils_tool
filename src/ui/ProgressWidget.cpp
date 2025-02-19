#include "ProgressWidget.hpp"

#include "BasicThread.hpp"
#include "DummyBagThread.hpp"
#include "EditBagThread.hpp"
#include "EncodingThread.hpp"
#include "MergeBagsThread.hpp"
#include "PublishImagesThread.hpp"
#include "PublishVideoThread.hpp"
#include "WriteToBagThread.hpp"
#include "WriteToImageThread.hpp"
#include "WriteToPCDsThread.hpp"

#include <QLabel>
#include <QMessageBox>
#include <QMovie>
#include <QProgressBar>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>

ProgressWidget::ProgressWidget(const QString& headerPixmapLabelTextBlack, const QString& headerPixmapLabelTextWhite,
                               const QString& headerLabelText, Utils::UI::InputParameters& parameters,
                               const int threadTypeId, QWidget *parent) :
    QWidget(parent)
{
    switch (threadTypeId) {
    case Utils::UI::TOOL_BAG_TO_VIDEO:
        m_thread = new EncodingThread(dynamic_cast<Utils::UI::VideoInputParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_VIDEO_TO_BAG:
        m_thread = new WriteToBagThread(dynamic_cast<Utils::UI::BagInputParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_BAG_TO_PCDS:
        m_thread = new WriteToPCDsThread(dynamic_cast<Utils::UI::AdvancedInputParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_BAG_TO_IMAGES:
        m_thread = new WriteToImageThread(dynamic_cast<Utils::UI::ImageInputParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_EDIT_BAG:
        m_thread = new EditBagThread(dynamic_cast<Utils::UI::EditBagInputParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_MERGE_BAGS:
        m_thread = new MergeBagsThread(dynamic_cast<Utils::UI::MergeBagsInputParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_DUMMY_BAG:
        m_thread = new DummyBagThread(dynamic_cast<Utils::UI::DummyBagInputParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_PUBLISH_VIDEO:
        m_thread = new PublishVideoThread(dynamic_cast<Utils::UI::PublishParameters&>(parameters), this);
        break;
    case Utils::UI::TOOL_PUBLISH_IMAGES:
        m_thread = new PublishImagesThread(dynamic_cast<Utils::UI::PublishParameters&>(parameters), this);
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
    if (threadTypeId != Utils::UI::TOOL_PUBLISH_VIDEO && threadTypeId != Utils::UI::TOOL_PUBLISH_IMAGES) {
        auto* const progressBar = new QProgressBar;
        progressBar->setVisible(false);
        progressWidget = progressBar;

        connect(m_thread, &BasicThread::progressChanged, this, [progressLabel, progressBar] (const QString& progressString, int progress) {
            if (!progressBar->isVisible()) {
                progressBar->setVisible(true);
            }
            progressBar->setValue(progress);
            progressLabel->setText(progressString);
        });
    } else {
        auto* const movie = new QMovie(isDarkMode? ":/gifs/publishing_white.gif" : ":/gifs/publishing_black.gif");
        movie->setScaledSize(QSize(120, 100));

        auto* const movieLabel = new QLabel;
        movieLabel->setMovie(movie);
        movieLabel->setAlignment(Qt::AlignHCenter);
        progressWidget = movieLabel;

        connect(m_thread, &BasicThread::progressChanged, this, [progressLabel] (const QString& progressString, int /* progress */) {
            progressLabel->setText(progressString);
        });
        connect(m_thread, &BasicThread::finished, this, [movie] {
            movie->stop();
        });

        movie->start();
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
    connect(m_thread, &BasicThread::openingCVInstanceFailed, this, [this] {
        auto* const messageBox = new QMessageBox(QMessageBox::Warning, "Failed processing files!",
                                                 "The file processing failed. Please make sure that all input parameters are set correctly, "
                                                 "that the input data is valid and disable the hardware acceleration, if necessary.");
        messageBox->exec();
        emit progressStopped();
    });
    connect(m_thread, &BasicThread::finished, this, [cancelButton, finishedButton] {
        cancelButton->setVisible(false);
        finishedButton->setVisible(true);
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
