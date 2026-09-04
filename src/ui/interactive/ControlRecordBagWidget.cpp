#include "ControlRecordBagWidget.hpp"

#include "UtilsGeneral.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QTimer>

#include <filesystem>

ControlRecordBagWidget::ControlRecordBagWidget(Parameters::RecordBagParameters& parameters, QWidget* parent)
    : ControlBagWidget(parameters, "Started Recording Bag File\n", ":/gifs/recording_bag", true, parent), m_parameters(parameters)
{
    m_controlsLayout->addWidget(m_playPauseButton);
    m_controlsLayout->addStretch();

    m_bagSizeLabel = new QLabel("");
    m_availableDiskSpaceLabel = new QLabel("");

    auto font = m_bagSizeLabel->font();
    font.setBold(true);
    m_bagSizeLabel->setFont(font);
    m_availableDiskSpaceLabel->setFont(font);

    m_upperLayout->insertSpacing(7, 10);
    m_upperLayout->insertWidget(8, m_bagSizeLabel);
    m_upperLayout->insertWidget(9, m_availableDiskSpaceLabel);
    m_upperLayout->insertSpacing(10, 10);
    m_upperLayout->insertWidget(11, m_loggerListWidget);
    m_upperLayout->setAlignment(m_bagSizeLabel, Qt::AlignHCenter);
    m_upperLayout->setAlignment(m_availableDiskSpaceLabel, Qt::AlignHCenter);

    m_bagRecorder = std::make_unique<BagRecorder>(parameters);

    // Simulate the first few terminal info messages displayed when calling ros2 bag record
    addRecordBagLoggerEntry("Press SPACE for pausing/resuming");
    addRecordBagLoggerEntry("Listening for topics...");
    m_loggerListWidget->setFocus();

    connect(m_bagRecorder.get(), &BagRecorder::recorderStarted, this, [this] {
        addRecordBagLoggerEntry("Event publisher thread: Starting.");
        addRecordBagLoggerEntry("Recording.");
    });
    connect(m_bagRecorder.get(), &BagRecorder::topicSubscribed, this, [this] (const QString& topic) {
        addRecordBagLoggerEntry("Subscribed to topic '" + topic + "'");
    });
    connect(m_bagRecorder.get(), &BagRecorder::allTopicsSubscribed, this, [this] {
        addRecordBagLoggerEntry("All requested topics are subscribed. Stopping discovery...");
    });

    m_loggerListWidget->setFocus();

    auto* const timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &ControlRecordBagWidget::updateSpaceInfoLabel);
    timer->start(1000);
}


void
ControlRecordBagWidget::updateSpaceInfoLabel()
{
    auto bagFileSpace = 0.0f;

    std::error_code errorCode;
    // Iterate over all files inside the bag to determine size
    for (const auto& entry : std::filesystem::recursive_directory_iterator(m_parameters.sourceDirectory.toStdString(),
                                                                           std::filesystem::directory_options::skip_permission_denied, errorCode)) {
        if (!std::filesystem::is_regular_file(entry, errorCode)) {
            continue;
        }
        bagFileSpace += static_cast<float>(std::filesystem::file_size(entry, errorCode));
    }
    // Show MB while less than 1 GB is used
    const auto sizeIndicator = bagFileSpace < Utils::General::GIGABYTE_IN_BYTES? " MB" : " GB";
    bagFileSpace /= bagFileSpace < Utils::General::GIGABYTE_IN_BYTES ? static_cast<float>(MEGABYTE_IN_BYTES)
                                                                     : static_cast<float>(Utils::General::GIGABYTE_IN_BYTES);
    m_bagSizeLabel->setText("Recorded Bag File Size: " + QString::number(bagFileSpace) + sizeIndicator);

    const auto remainingSpace = Utils::General::getAvailableDriveSpace(m_parameters.sourceDirectory);
    m_availableDiskSpaceLabel->setText("Available Disk Space: " + QString::number(remainingSpace) + "GB");
}
