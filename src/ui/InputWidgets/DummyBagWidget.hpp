#pragma once

#include "DummyBagSettings.hpp"
#include "Parameters.hpp"
#include "TopicListingInputWidget.hpp"

#include <QPointer>
#include <QWidget>

class DummyTopicWidget;

class QFormLayout;
class QSpinBox;

// Widget used to manage creating a ROS bag with dummy data
class DummyBagWidget : public TopicListingInputWidget
{
    Q_OBJECT

public:
    DummyBagWidget(Parameters::DummyBagParameters& parameters,
                   bool                            checkROS2NameConform,
                   QWidget*                        parent = 0);

private slots:
    void
    removeDummyTopicWidget();

    void
    createNewDummyTopicWidget(const Parameters::DummyBagParameters::DummyBagTopic& topics,
                              int                                                  index);

    void
    useCustomRateCheckBoxPressed(int state);

private:
    std::optional<bool>
    areTopicsValid() override;

private:
    QVector<QPointer<DummyTopicWidget> > m_dummyTopicWidgets;

    QPointer<QFormLayout> m_formLayout;
    QPointer<QSpinBox> m_rateSpinBox;

    Parameters::DummyBagParameters& m_parameters;

    DummyBagSettings m_settings;

    const bool m_checkROS2NameConform;

    static constexpr int MAXIMUM_NUMBER_OF_TOPICS = 4;
};
