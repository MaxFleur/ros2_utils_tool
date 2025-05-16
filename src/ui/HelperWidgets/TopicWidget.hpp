#pragma once

#include <QLineEdit>
#include <QPointer>
#include <QWidget>

class QToolButton;

// Small helper widget used to select a topic type for the ROS bag dummy creation
class TopicWidget : public QWidget
{
    Q_OBJECT

public:
    TopicWidget(bool           isDummyWidget,
                bool           addRemoveButton,
                const QString& topicTypeText = "",
                const QString& topicNameText = "",
                QWidget*       parent = 0);

    const QString
    getTopicName() const
    {
        return m_topicNameLineEdit->text();
    }

signals:
    void
    topicTypeChanged(QString type);

    void
    topicNameChanged(QString name);

    void
    topicRemoveButtonClicked();

private:
    void
    setPixmapLabelIcon() const;

    bool
    event(QEvent *event);

private:
    QPointer<QLineEdit> m_topicNameLineEdit;
    QPointer<QToolButton> m_removeTopicButton;
};
