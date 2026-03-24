#pragma once

#include "BagRecorder.hpp"
#include "ControlBagWidget.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QPointer>
#include <QWidget>

class QLabel;

// Widget used to control playing a bag file.
class ControlRecordBagWidget : public ControlBagWidget
{
    Q_OBJECT

public:
    ControlRecordBagWidget(Parameters::RecordBagParameters& parameters,
                           QWidget*                         parent = 0);

private:
    void
    updateSpaceInfoLabel();

    void
    handleBagControlInstance() override
    {
        m_bagRecorder->toggleState(m_isActive);
    }

    inline void
    addRecordBagLoggerEntry(const QString& entryText)
    {
        addLoggerWidgetEntry(Utils::ROS::getCurrentROSTimeAsString() + " " + entryText);
    }

private:
    std::unique_ptr<BagRecorder> m_bagRecorder;

    QPointer<QLabel> m_bagSizeLabel;
    QPointer<QLabel> m_availableDiskSpaceLabel;

    const Parameters::RecordBagParameters& m_parameters;

    static constexpr long MEGABYTE_IN_BYTES = 1048576;
};
