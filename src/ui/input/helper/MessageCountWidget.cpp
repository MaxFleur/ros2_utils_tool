#include "MessageCountWidget.hpp"

#include <QHBoxLayout>
#include <QLabel>

MessageCountWidget::MessageCountWidget(int minimum, int maximum, int currentMaximumValue, QWidget *parent) :
    QWidget(parent)
{
    m_lowerBox = new QSpinBox;
    // Start with a value of 0 instead one 1, most users should be able to handle that :-P
    m_lowerBox->setRange(0, maximum);
    m_lowerBox->setValue(minimum);

    m_upperBox = new QSpinBox;
    m_upperBox->setRange(1, maximum);
    m_upperBox->setValue(currentMaximumValue);

    auto* const rangeDifferenceLabel = new QLabel("-");

    auto* const mainLayout = new QHBoxLayout;
    mainLayout->addWidget(m_lowerBox);
    mainLayout->addStretch();
    mainLayout->addWidget(rangeDifferenceLabel);
    mainLayout->addStretch();
    mainLayout->addWidget(m_upperBox);
    // Set these margins because we are going to integrate the widget into existing widgets
    // where any extra widgets would create undesired extra space
    mainLayout->setContentsMargins(0, 0, 0, 0);

    setLayout(mainLayout);

    connect(m_lowerBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        emit lowerValueChanged(value);
    });
    connect(m_upperBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this] (int value) {
        emit upperValueChanged(value);
    });
}
