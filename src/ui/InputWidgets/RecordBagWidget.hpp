#pragma once

#include "Parameters.hpp"
#include "RecordBagSettings.hpp"
#include "TopicListingInputWidget.hpp"

#include <QPointer>

class QFormLayout;
class QLineEdit;
class QWidget;

// Widget used to manage recording a bag file
class RecordBagWidget : public TopicListingInputWidget
{
    Q_OBJECT

public:
    RecordBagWidget(Parameters::RecordBagParameters& parameters,
                    QWidget*                         parent = 0);

private slots:
    void
    removeLineEdit();

    void
    createNewTopicLineEdit(const QString& topicName,
                           int            index);

private:
    std::optional<bool>
    areTopicsValid() override;

private:
    QVector<QPointer<QLineEdit> > m_topicLineEdits;

    QPointer<QFormLayout> m_topicsFormLayout;

    Parameters::RecordBagParameters& m_parameters;

    RecordBagSettings m_settings;
};
