#pragma once

#include <QPointer>
#include <QSpinBox>
#include <QWidget>

// Display the lower and upper messages which should be extracted out of an existing ROS bag
// into a newly edited one
class MessageCountWidget : public QWidget
{
    Q_OBJECT

public:
    MessageCountWidget(int      minimum,
                       int      maximum,
                       int      currentMaximumValue,
                       QWidget* parent = 0);

signals:
    void
    lowerValueChanged(int value);

    void
    upperValueChanged(int value);

public:
    int
    getLowerValue() const
    {
        return m_lowerBox->value();
    }

    int
    getHigherValue() const
    {
        return m_upperBox->value();
    }

private:
    QPointer<QSpinBox> m_lowerBox;
    QPointer<QSpinBox> m_upperBox;
};
