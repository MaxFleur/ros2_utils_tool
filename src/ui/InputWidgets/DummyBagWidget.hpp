#pragma once

#include "DummyBagSettings.hpp"
#include "Parameters.hpp"
#include "TopicListingInputWidget.hpp"

#include <QPointer>
#include <QWidget>

class QSpinBox;

// Widget used to manage creating a ROS bag with dummy data
class DummyBagWidget : public TopicListingInputWidget
{
    Q_OBJECT

public:
    DummyBagWidget(Parameters::DummyBagParameters& parameters,
                   bool                            warnROS2NameConvention,
                   QWidget*                        parent = 0);

private slots:
    void
    removeDummyTopicWidget(int row);

    void
    createNewDummyTopicWidget(const Parameters::DummyBagParameters::DummyBagTopic& topics,
                              int                                                  index);

    void
    useCustomRateCheckBoxPressed(int state);

private:
    std::optional<bool>
    areTopicsValid() const override;

private:
    QPointer<QSpinBox> m_rateSpinBox;

    Parameters::DummyBagParameters& m_parameters;

    DummyBagSettings m_settings;

    const bool m_warnROS2NameConvention;

    static constexpr int TOPIC_WIDGET_OFFSET = 3;
    static constexpr int MAXIMUM_NUMBER_OF_TOPICS = 5;
};
