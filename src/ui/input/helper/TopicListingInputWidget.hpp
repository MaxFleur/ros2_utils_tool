#pragma once

#include "BasicInputWidget.hpp"
#include "BasicSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

#include <optional>

class TopicWidget;

class QFormLayout;
class QHBoxLayout;
class QLabel;
class QToolButton;

// A widget showing widgets to add and remove topics
class TopicListingInputWidget : public BasicInputWidget
{
    Q_OBJECT
public:
    explicit
    TopicListingInputWidget(Parameters::BasicParameters& parameters,
                            const QString&               titleText,
                            const QString&               iconText,
                            const QString&               settingsIdentifierText,
                            QWidget*                     parent = 0);

protected slots:
    void
    sourceButtonPressed();

    void
    okButtonPressed() const;

protected:
    virtual std::optional<bool>
    areTopicsValid() const = 0;

    // Have to overwrite this one because we are using more additional icons then just the top one
    void
    setPixmapLabelIcon() const;

    bool
    event(QEvent *event);

protected:
    // A QFormLayout provides no easy access to layout labels, so we keep them
    // in an extra vector for renaming after a row was removed
    QVector<QPointer<QLabel> > m_topicLabels;
    QVector<QPointer<TopicWidget> > m_topicWidgets;

    QPointer<QToolButton> m_addTopicButton;

    QPointer<QFormLayout> m_formLayout;
    QPointer<QHBoxLayout> m_topicButtonLayout;

    int m_numberOfTopics = 0;

private:
    Parameters::BasicParameters& m_parameters;

    BasicSettings m_settings;
};
