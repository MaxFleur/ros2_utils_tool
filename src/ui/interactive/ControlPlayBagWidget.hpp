#pragma once

#include "BagPlayer.hpp"
#include "ControlBagWidget.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <QWidget>

// Widget used to control playing a bag file.
class ControlPlayBagWidget : public ControlBagWidget
{
    Q_OBJECT

public:
    ControlPlayBagWidget(Parameters::PlayBagParameters& parameters,
                         QWidget*                       parent = 0);

private slots:
    void
    decreaseRate();

    void
    increaseRate();

    void
    playNextMessage();

private:
    void
    handleBagControlInstance() override
    {
        m_bagPlayer->toggleState(m_isActive);
    }

    inline void
    addPlayBagLoggerEntry(const QString& entryText)
    {
        addLoggerWidgetEntry(Utils::ROS::getCurrentROSTimeAsString() + " " + entryText);
    }

private:
    std::unique_ptr<BagPlayer> m_bagPlayer;

    double m_currentRate;
};
