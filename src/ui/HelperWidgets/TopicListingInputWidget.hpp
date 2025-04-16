#pragma once

#include "BasicInputWidget.hpp"
#include "BasicSettings.hpp"
#include "Parameters.hpp"

#include <QPointer>
#include <QWidget>

#include <optional>

class QHBoxLayout;
class QToolButton;
class QVBoxLayout;

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
    QPointer<QToolButton> m_removeTopicButton;
    QPointer<QToolButton> m_addTopicButton;

    QPointer<QHBoxLayout> m_topicButtonLayout;
    QPointer<QVBoxLayout> m_controlsLayout;

    int m_numberOfTopics = 0;

private:
    Parameters::BasicParameters& m_parameters;

    BasicSettings m_settings;
};
