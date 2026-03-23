#pragma once

#include "BagRecorder.hpp"
#include "ControlBagWidget.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QWidget>

// Widget used to control playing a bag file.
class ControlRecordBagWidget : public ControlBagWidget
{
    Q_OBJECT

public:
    ControlRecordBagWidget(Parameters::RecordBagParameters& parameters,
                           QWidget*                         parent = 0);

private:
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
};
