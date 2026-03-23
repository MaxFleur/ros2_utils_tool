#include "ControlRecordBagWidget.hpp"

#include <QHBoxLayout>
#include <QListWidget>

ControlRecordBagWidget::ControlRecordBagWidget(Parameters::RecordBagParameters& parameters, QWidget* parent)
    : ControlBagWidget(parameters, "Started recording Bag File\n", ":/icons/tools/record_bag_", true, parent)
{
    m_controlsLayout->addWidget(m_playPauseButton);
    m_controlsLayout->addStretch();

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
}
